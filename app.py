#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import time
import requests
import logging
import sys
import json
import board
import digitalio
import adafruit_max31865
import RPi.GPIO as GPIO

lastGet = 0
lastSent = 0
lastFermenterRun = 0
forceSendingData = False

scriptDir = os.path.dirname(__file__)
configPath = os.path.join(scriptDir, 'config/app.json')
configuration = {}

spi = board.SPI()
tempSensorList = {}
tempSensorBoardMap = {}
boardData = {}
heatResistorPwm = {}
plannerTimer = {}


def initHeatUnit(unitId):
    global heatResistorPwm

    unitData = boardData['unitList'][unitId]

    GPIO.setwarnings(False) # disable warnings
    GPIO.setmode(GPIO.BCM) # set pin numbering system #GPIO.BCM
    GPIO.setup(unitData['resistor'], GPIO.OUT)
    pwm = GPIO.PWM(unitData['resistor'], 50)
    pwm.start(0) # off
    heatResistorPwm[unitId] = pwm

def initFermenter(unitId):
    global boardData;
    unitData = boardData['unitList'][unitId]
    GPIO.setwarnings(False) # disable warnings
    GPIO.setmode(GPIO.BCM) # set pin numbering system #GPIO.BCM
    GPIO.setup(unitData['cool'], GPIO.OUT)
    GPIO.setup(unitData['heat'], GPIO.OUT)
    unitData['state'] = 0

def setHeatPlanStatus(unitId, status):
    global boardData
    global plannerTimer
    global forceSendingData

    STATUS_STOPPED = 0
    STATUS_RUNNING = 1
    STATUS_PAUSE = 2
    STATUS_RESET = 3
    STATUS_GO_TO = 4
    STATUS_WAIT_TARGET = 5
    ASK_OFF = 0

    if status == STATUS_RUNNING:
        boardData['unitList'][unitId]['planner']['currentStatus'] = STATUS_WAIT_TARGET
        plannerTimer[unitId] = time.time() - boardData['unitList'][unitId]['planner']['currentTime'] * 60
        boardData['unitList'][unitId]['sendMode'] = True
        forceSendingData = True
    elif status == STATUS_PAUSE:
        boardData['unitList'][unitId]['planner']['currentStatus'] = STATUS_PAUSE
        boardData['unitList'][unitId]['mode'] = 'off'
        boardData['unitList'][unitId]['power'] = -1
        boardData['unitList'][unitId]['target'] = -1
        boardData['unitList'][unitId]['sendMode'] = True
        forceSendingData = True
    elif status == STATUS_RESET:
        boardData['unitList'][unitId]['planner']['currentStatus'] = STATUS_STOPPED
        boardData['unitList'][unitId]['planner']['currentStep'] = 0
        boardData['unitList'][unitId]['planner']['currentTime'] = 0
        boardData['unitList'][unitId]['mode'] = 'off'
        boardData['unitList'][unitId]['power'] = -1
        boardData['unitList'][unitId]['target'] = -1
        boardData['unitList'][unitId]['sendMode'] = True
        forceSendingData = True


def runHeatPlan(unitId):
    global boardData
    global plannerTimer
    global forceSendingData

    STATUS_STOPPED = 0
    STATUS_RUNNING = 1
    STATUS_PAUSE = 2
    STATUS_RESET = 3
    STATUS_GO_TO = 4
    STATUS_WAIT_TARGET = 5
    ASK_OFF = 0
    ACTION_OFF = 0
    ACTION_TARGET = 1
    ACTION_BOIL = 2

    unitData = boardData['unitList'][unitId]
    planner = boardData['unitList'][unitId]['planner']

    boardData['unitList'][unitId]['sendMode'] = False
    forceSendingData = True


    # Ask to change state
    if boardData['unitList'][unitId]['planner']['askStatus'] == STATUS_RUNNING:
        setHeatPlanStatus(unitId, STATUS_RUNNING);
    elif boardData['unitList'][unitId]['planner']['askStatus'] == STATUS_PAUSE:
        setHeatPlanStatus(unitId, STATUS_PAUSE);
    elif boardData['unitList'][unitId]['planner']['askStatus'] == STATUS_RESET:
        setHeatPlanStatus(unitId, STATUS_RESET);

    # Ask to change step
    if boardData['unitList'][unitId]['planner']['askGoTo'] >= 0:
        plannerTimer[unitId] = time.time()
        boardData['unitList'][unitId]['planner']['currentTime'] = 0
        boardData['unitList'][unitId]['planner']['currentStep'] = boardData['unitList'][unitId]['planner']['askGoTo'];

    # If plan is not running
    if boardData['unitList'][unitId]['planner']['currentStatus'] != STATUS_RUNNING and boardData['unitList'][unitId]['planner']['currentStatus'] != STATUS_WAIT_TARGET:
        return

    # If timer is not started
    if unitId not in plannerTimer:
        plannerTimer[unitId] = time.time()

    # Current Step
    currentStep = boardData['unitList'][unitId]['planner']['stepList'][boardData['unitList'][unitId]['planner']['currentStep']]

    if boardData['unitList'][unitId]['planner']['currentStatus'] == STATUS_WAIT_TARGET:
        if currentStep['action'] == ACTION_BOIL:
            # If step is boil, wait temperature is 98
            # @TODO 98 must be a configurable value
            if boardData['unitList'][unitId]['temperature'] >= 98:
                boardData['unitList'][unitId]['planner']['currentStatus'] = STATUS_RUNNING
                plannerTimer[unitId] = time.time()
            else:
                # The target is not reached, reset time
                plannerTimer[unitId] = time.time()
        elif currentStep['action'] == ACTION_TARGET:
            if boardData['unitList'][unitId]['temperature'] >= formatNullableInteger(currentStep['target']):
                # Target is reached, reset time and change status
                boardData['unitList'][unitId]['planner']['currentStatus'] = STATUS_RUNNING
                plannerTimer[unitId] = time.time()
            else:
                # The target is not reached, reset time
                plannerTimer[unitId] = time.time()

    # Update current time
    boardData['unitList'][unitId]['planner']['currentTime'] = int((time.time() - plannerTimer[unitId]) / 60)


    # If step is ended
    if boardData['unitList'][unitId]['planner']['currentTime'] >= currentStep['duration']:
        if len(boardData['unitList'][unitId]['planner']['stepList']) -1 == boardData['unitList'][unitId]['planner']['currentStep']:
            # No other step => stop the plan
            setHeatPlanStatus(unitId, STATUS_RESET)
            return
        elif currentStep['onFinishGoNext']:
            # Go to the next step
            boardData['unitList'][unitId]['planner']['currentStep'] += 1
            boardData['unitList'][unitId]['planner']['currentTime'] = 0
            boardData['unitList'][unitId]['planner']['currentStatus'] = STATUS_WAIT_TARGET
            plannerTimer[unitId] = time.time()
            currentStep = boardData['unitList'][unitId]['planner']['stepList'][boardData['unitList'][unitId]['planner']['currentStep']]
        else:
            # Go to the next step, set time to 0 and pause
            boardData['unitList'][unitId]['planner']['currentStep'] += 1
            boardData['unitList'][unitId]['planner']['currentTime'] = 0
            setHeatPlanStatus(unitId, STATUS_PAUSE)
            return

    if currentStep is None:
        setHeatPlanStatus(unitId, STATUS_RESET)
    else:
        if currentStep['action'] == ACTION_TARGET:
            boardData['unitList'][unitId]['mode'] = 'temperature'
            boardData['unitList'][unitId]['power'] = formatNullableInteger(currentStep['power'])
            boardData['unitList'][unitId]['target'] = formatNullableInteger(currentStep['target'])
        elif currentStep['action'] == ACTION_BOIL:
            boardData['unitList'][unitId]['mode'] = 'power'
            boardData['unitList'][unitId]['power'] = currentStep['power']
            boardData['unitList'][unitId]['target'] = -1
        else:
            boardData['unitList'][unitId]['mode'] = 'off'
            boardData['unitList'][unitId]['power'] = -1
            boardData['unitList'][unitId]['target'] = -1
        boardData['unitList'][unitId]['sendMode'] = True
        forceSendingData = True


def formatNullableInteger(number):
    if number == 0:
        return -1
    return number


def runHeatUnit(unitId):
    unitData = boardData['unitList'][unitId]
    temperature = unitData['temperature']

    runHeatPlan(unitId);

    power = 0
    if unitData['mode'] == 'temperature':
        if unitData['power'] and unitData['target']:
            if temperature < unitData['target'] - unitData['t1']:
                power = unitData['power']
            elif temperature < unitData['target'] - unitData['t2']:
                power = unitData['power'] / 100 * ((unitData['target'] - temperature) / (unitData['t1'])) * 100
    elif unitData['mode'] == 'power':
        power = unitData['power']

    pwm = heatResistorPwm[unitId]

    pwm.ChangeDutyCycle(float(power))

    return power

def runFermenter(unitId):
    global lastFermenterRun
    unitData = boardData['unitList'][unitId]

    if 'state' in unitData:
        currentState = unitData['state']
    else:
        currentState = 0

    lastState = currentState

    currentTime = time.time()
    if currentTime - lastFermenterRun < 60:
        return currentState

    lastFermenterRun = currentTime

    fermenterTemperature = unitData['temperature']

    if unitData['mode'] == 'off':
         currentState = 0
    elif unitData['mode'] == 'temperature':
        if fermenterTemperature > unitData['target'] + unitData['t1']:
            currentState = -1
        elif currentState == -1 and fermenterTemperature > unitData['target']:
            currentState = -1
        elif fermenterTemperature < unitData['target'] - unitData['t2']:
            currentState = 1
        elif currentState == 1 and fermenterTemperature < unitData['target']:
            currentState = 1
        else:
            currentState = 0
    elif unitData['mode'] == 'heat':
        currentState = 1
    elif unitData['mode'] == 'cool':
        currentState = -1

    if currentState != lastState:
        if currentState == -1:
            GPIO.output(unitData['cool'], False) # False = on
            GPIO.output(unitData['heat'], True) # True = off
        elif currentState == 1:
            GPIO.output(unitData['heat'], False) # False = on
            GPIO.output(unitData['cool'], True) # True = off
        else:
            GPIO.output(unitData['cool'], True) # True = off
            GPIO.output(unitData['heat'], True) # True = off

    unitData['state'] = currentState;

    return currentState

def initTemperatureSensor():
    global tempSensorBoardMap
    global tempSensorList
    for unitId in boardData['unitList']:
        if boardData['unitList'][unitId]['type'] == 'HeatUnit' or boardData['unitList'][unitId]['type'] == 'FermenterUnit':
            if 'tempSensor' in boardData['unitList'][unitId] and boardData['unitList'][unitId]['tempSensor'] is not None:
                cs = digitalio.DigitalInOut(getattr(board, 'D'+str(boardData['unitList'][unitId]['tempSensor'])));
                tempSensorList[boardData['unitList'][unitId]['tempSensor']] =  adafruit_max31865.MAX31865(spi, cs, rtd_nominal=100, ref_resistor=430.0, wires=2)
                if boardData['unitList'][unitId]['tempSensor'] not in tempSensorBoardMap:
                    tempSensorBoardMap[boardData['unitList'][unitId]['tempSensor']] = [];

                tempSensorBoardMap[boardData['unitList'][unitId]['tempSensor']].append(unitId)

def loadConfiguration():
    global configuration

    configurationFile = open(configPath)
    try:
        configuration = json.load(configurationFile)
        configurationFile.close();
    except json.decoder.JSONDecodeError:
        print('config/app.json Invalid JSON syntax')
        exit

def loadBoardData():
    global lastGet
    global boardData

    currentTime = time.time()
    if currentTime - lastGet < 5:
        return

    lastGet = currentTime
    boardData = getApiBoardData()


def collectTemperature():
    global boardData
    # logging.basicConfig(filename=os.path.join(scriptDir, 'log/debug.log'), level=logging.DEBUG)
    logging.basicConfig(filename=os.path.join(scriptDir, 'log/error.log'), level=logging.ERROR)

    if False == os.path.isfile(configPath) or os.stat(configPath).st_size == 0:
        print('Missing config/app.json')
        exit


    for tempSensorPin in tempSensorList:
        temperature = tempSensorList[tempSensorPin].temperature
        for unitId in tempSensorBoardMap[tempSensorPin]:
            boardData['unitList'][unitId]['temperature'] = temperature

def getSendParams():
    params = {
        'token': configuration['token'],
        'boardId': configuration['boardId']
    }

    for unitId in boardData['unitList']:
        if 'temperature' in boardData['unitList'][unitId] :
            #params['unitList'][unitId] = {'temperature': boardData['unitList'][unitId]['temperature']}
            params['unitList['+unitId+'][temperature]'] = boardData['unitList'][unitId]['temperature']
        if 'state' in boardData['unitList'][unitId] :
            params['unitList['+unitId+'][state]'] = boardData['unitList'][unitId]['state']
        if 'planner' in boardData['unitList'][unitId] :
            params['unitList['+unitId+'][planner][currentStatus]'] = boardData['unitList'][unitId]['planner']['currentStatus']
            params['unitList['+unitId+'][planner][currentTime]'] = boardData['unitList'][unitId]['planner']['currentTime']
            params['unitList['+unitId+'][planner][currentStep]'] = boardData['unitList'][unitId]['planner']['currentStep']
        if 'sendMode' in boardData['unitList'][unitId] and boardData['unitList'][unitId]['sendMode']:
            params['unitList['+unitId+'][mode]'] = boardData['unitList'][unitId]['mode']
            if boardData['unitList'][unitId]['target'] is not None:
                params['unitList['+unitId+'][target]'] = boardData['unitList'][unitId]['target']
            if boardData['unitList'][unitId]['power'] is not None:
                params['unitList['+unitId+'][power]'] = boardData['unitList'][unitId]['power']

    return params

def sendData():
    global lastSent
    global forceSendingData

    currentTime = time.time()
    if currentTime - lastSent < 2 and False == forceSendingData:
        return

    lastSent = currentTime

    headers = {'Accept-Charset': 'UTF-8'}
    params = getSendParams()
    url = configuration['domain'] + 'user/board/set.php';
    response = (requests.post(url, data=params, headers=headers, verify=configuration['verifySSL'])).json()

    if False == response['status']:
        logging.error('=== Send Data Error ===')
        logging.error(response['reason'])
        logging.error(response)
        logging.error('=======================')
    else:
        forceSendingData = False

def getApiBoardData():
    headers = {'Accept-Charset': 'UTF-8'}
    params = {
        'token': configuration['token']
    }

    url = configuration['domain'] + 'user/check-token.php';
    response = (requests.post(url, data=params, headers=headers, verify=configuration['verifySSL'])).json()

    if False == response['status']:
        logging.error('=== Check Token Error ===')
        logging.error(response['reason'])
        logging.error('===========================')
        exit

    headers = {'Accept-Charset': 'UTF-8'}
    params = {
        'token': configuration['token'],
        'boardId': configuration['boardId']
    }
    url = configuration['domain'] + 'user/board/get.php';
    response = (requests.post(url, data=params, headers=headers, verify=configuration['verifySSL'])).json()
    # logging.error('=== Debug Api Board Get Response ===')
    # logging.error(response)
    # logging.error('====================================')

    if False == response['status']:
        logging.error('=== Api Board Get Error ===')
        logging.error(response['reason'])
        logging.error('===========================')
        exit

    boardData = response['data']

    return boardData

def app():
    loadConfiguration()
    loadBoardData()
    initTemperatureSensor()
    collectTemperature()
    for unitId in boardData['unitList']:
        if boardData['unitList'][unitId]['type'] == 'HeatUnit':
            initHeatUnit(unitId)
        elif boardData['unitList'][unitId]['type'] == 'FermenterUnit':
            initFermenter(unitId)

    while True:
        for unitId in boardData['unitList']:
            if boardData['unitList'][unitId]['type'] == 'HeatUnit':
                boardData['unitList'][unitId]['state'] = int(runHeatUnit(unitId))
            elif boardData['unitList'][unitId]['type'] == 'FermenterUnit':
                boardData['unitList'][unitId]['state'] = runFermenter(unitId)


        time.sleep(2)

        collectTemperature()
        sendData()

        loadBoardData()

app()
