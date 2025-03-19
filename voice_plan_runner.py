import speech_recognition as sr
import os 
import delivery_plan_runner_pi5_base as HTN
import requests
import json
import re
import gtts
import sys
import pickle
import sounddevice
from playsound import playsound  
import time
import datetime
import speech_engine

def writeToFile(fileName, line):
    file = open(fileName, "a")
    file.write("\n" + str(datetime.datetime.now()) + ',' + line)
    file.close()

class LLMConnector: #class used to create calls to Ollama API
    def __init__(self, LLM, systemMessage): #takes the name and a system message as input and creates the dictionary needed to access the llm
        self.url = "http://localhost:11434/api/generate" #local ollama port
        self.headers = {
            "Content-Type": "application/json"
        }
        self.data = {
            "model": LLM,
            "prompt": "",
            "stream": False,
            "system": systemMessage
        }

    def prompt(self, promptMessage): #takes prompt as input, updates dictionary, and creates call to llm through ollama
        self.data["prompt"] = promptMessage 
        start = time.perf_counter()
        response = requests.post(self.url, headers=self.headers, data=json.dumps(self.data)) #posts request to ollama API and recieves response
        stop = time.perf_counter()
        timer = stop-start
        if response.status_code == 200: #code for success
            response_text = response.text
            response = json.loads(response.text)["response"] #extracts response from json object
            writeToFile("transcript.txt", "llm: " + response)
            writeToFile("transcript.txt", "**Response recieved in " + str(timer) + " seconds**")
            return response
        else:
            print( "API Error:", response.status_code, response.text)
            return None

def outputSpeech(text): #method for text to voice output, takes message as input
    speech_engine.pyttsSpeaker().outputSpeech(text)
    print(text)
    writeToFile('transcript.txt', "system: " + text)
def getSpeechInput(output): #outputs message, returns result of voice input, takes message as input
    input = speech_engine.voskRecognizer().getInput(output)
    if len(output) >= 1:
        writeToFile("transcript.txt", "system: " + output)
    writeToFile("transcript.txt", "user: " + input)
    return input

def getStateDescription(runner):
    description = "The following is a description of the destinations the robot can travel to " + runner.state.description
    if len(runner.state.package_locations) > 0:
        description += " The following is a description of where each package is located. "
        for package in runner.state.package_locations:
            description += package + " is located at " + runner.state.package_locations[package] + ". "
    else:
        description += " There are no packages in the system."
    return description

class PackageDeliveryState(): #state in which robot delivers package from current location to a designated destination
    def __init__(self, runner, request, startTime): #initiated with runner
        self.runner = runner
        self.request= request
        self.startTime = startTime
        #Sets up instance of object that is used to generate method calls through ollama API with phi3:3.8b as the model and a description of the floor and task as a system message
        message = getStateDescription(self.runner) + " Based on the following input, you are to identify a package and the destination the user would like the package to be delivered to. Output your findings in the following format **package** **destination**, replacing **package** with the name of the package and **destination** with the name of the destination, as identified from the input."
        self.methodCaller = LLMConnector("phi3:3.8b", message)
        #Sets up instance of object that is used to evaluate user verification through ollama API with phi3:3.8b as the model and a description of the floor and task as a system message
        self.classifier = LLMConnector("phi3:3.8b", "You are an expert classifier who determines if the prompt is a positive or negative response. If it is a positive response, output a 1. If it is a negative response or you are unsure, output a 0. Do not include any additional text, explanations, or notes.")
    
    def action(self): #action prompts for instructions and generates plan
        state = "delivery"
        userRevisions = 0
        systemCorrections = 0
        executed = False
        prompt = self.request
        timer = ""
        for i in range(2): #allows additional attempt to fine tune prompt before failing process
            deliveryDetails = self.methodCaller.prompt(prompt).replace('*','')  #recieves details in specifed format from llm
            parts = deliveryDetails.split() #seperates location and item
            if len(parts) >= 2 and parts[0].lower() in self.runner.state.package_locations and parts[1][0:2] in self.runner.state.graph: #verifies that method call contains valid package and location
                response = getSpeechInput("To confirm, would you like " + parts[0].lower() + " to be delivered to " + self.runner.state.aliases[parts[1][0:2]] + "?") #verifies request using package and location
                classification = self.classifier.prompt(response) #recieves a 0 or 1 as a response from llm- 1 indicates positive verification
                if '1' in classification: #user has verified method call
                    self.runner.current_input = "deliver " + parts[0].lower() + " " + parts[1][0:2]  #sets method as runner's current input
                    self.runner.deliver() #executes method call
                    self.runner.run_loop() #runs loop to execute plan
                    timer = str(time.perf_counter()-self.startTime)
                    writeToFile("transcript.txt", "**Command executed in " + str(timer) + " seconds**")
                    executed = True
                    break #breaks loop because no further fine tuning is needed
                elif i == 1: #user has not verified prompt and all attempts are used
                    outputSpeech("Unable to verify instructions, process failed")
                else: #user has not verified prompt, but attempts remain
                    newInfo = getSpeechInput("Please clarify an item and destination") #gets clarification from user                  
                    prompt = prompt + newInfo #adds additional instructions to original prompt
                    userRevisions += 1
            elif i <1: #tries again with same prompt because the response from the llm was bad
                newInfo = getSpeechInput("Please clarify an item and destination") #gets clarification from user                  
                prompt = prompt + newInfo #adds additional instructions to original prompt
                userRevisions += 1
                systemCorrections += 1
            else:
                outputSpeech("Process failed")
        writeToFile("log.txt", state + "," + str(userRevisions) + "," + str(systemCorrections) + "," + str(executed) + "," + str(timer) + ",voice")
        return RoutingState(self.runner) #returns next state to main method, which is the routing state

class DescriptionState(): #state in which llm provides description of state of system
    def __init__(self, runner, startTime): #initialized with runner
        self.runner = runner
        self.startTime = startTime
        message = getStateDescription(self.runner) + "Based only on the provided information, briefly describe where each package is currently located in plain english. Do not provide additional explanations or speculation. Do not make up or describe any packages that are not explicitly listed with a location in the prompt." 
        #creates instance of LLM Connector that sets up model to recieve a list of current locations and describe the sytem
        self.describer = LLMConnector("phi3:instruct", message)
    def action(self): #action outputs a description of the state of the system
        state = "description"
        userRevisions = 0
        systemCorrections = 0
        executed = True
        timer = ""
        outputSpeech(self.describer.prompt("Provide a short description.")) #creates call to llm with all locations and outputs resulting description
        timer = str(time.perf_counter()-self.startTime)
        writeToFile("transcript.txt", "**Command executed in " + str(timer) + " seconds**")
        writeToFile("log.txt", state + "," + str(userRevisions) + "," + str(systemCorrections) + "," + str(executed) + "," + str(timer))
        return RoutingState(self.runner) #returns next state to main method, which is the routing state

class QuestionState(): #state in which the user can ask a question for clarification
    def __init__(self, runner, request, startTime): #initialized with state and planner
        self.runner = runner
        self.request = request
        self.startTime = startTime
        message = "You are part of an artificial intelligence system that controls the movement of an iRobot Create3 robot. The robot can be navigated between any two destinations and deliver packages. " + getStateDescription(self.runner) + " The following is a question asked by the user. Do your best to provide a brief response based on the previous information. "
        #creates local instance of connector that describes the premise of the system and sets the llm up to recieve a list of locations and a question to answer
        self.answerer = LLMConnector("phi3:instruct", message)
    def action(self): #action gets question from user and outputs response
        state = "question"
        userRevisions = 0
        systemCorrections = 0
        executed = True
        prompt = self.request
        timer = ""
        outputSpeech(self.answerer.prompt("Question to be answered: " + self.request)) #prompts and gets response from llm, outputs reponse
        timer = str(time.perf_counter()-self.startTime)
        writeToFile("transcript.txt", "**Command executed in " + str(timer) + " seconds**")
        writeToFile("log.txt", state + "," + str(userRevisions) + "," + str(systemCorrections) + "," + str(executed) + "," + str(timer))
        return RoutingState(self.runner) #returns next state to main method, which is the routing state

class NavigationState(): #state in which the robot moves from current location to new location
    def __init__(self, runner, request, startTime): #initialized with state and runner
        self.runner = runner
        self.request = request
        self.startTime = startTime
        message = getStateDescription(self.runner) + " Based on the following input, you are to identify the name of the destination the user would like the robot to navigate to and output it in the following format **destination** by replacing **destination** with the one identified." 
        #Sets up instance of object that is used to generate method calls through ollama API with phi3:3.8b as the model and a description of the floor and task as a system message
        self.methodCaller = LLMConnector("phi3:3.8b", message)
        #Sets up instance of object that is used to evaluate user verification through ollama API with phi3:3.8b as the model 
        self.classifier = LLMConnector("phi3:3.8b", "You are an expert classifier who determines if the prompt is a positive or negative response. If it is a positive response, output a 1. If it is a negative response or you are unsure, output a 0. Do not include any additional text, explanations, or notes.")
    
    def action(self): #action gets location from user and generates plan for robot to travel to location
        state = "navigation"
        userRevisions = 0
        systemCorrections = 0
        executed = False
        prompt = self.request
        timer = ""
        prompt = self.request
        for i in range(2): #allows additional attempt to fine tune prompt before failing process
            navigationDetails = self.methodCaller.prompt(prompt).replace('*','')  #recieves details in specifed format from llm
            parts = navigationDetails.split() #seperates location and item
            if len(parts) >= 1 and parts[0][0:2] in self.runner.state.graph: #verifies that method call contains valid location
                response = getSpeechInput("To confirm, would you like the robot to navigate to" + self.runner.state.aliases[parts[0]] + "?") #verifies request using location
                classification = self.classifier.prompt(response) #recieves a 0 or 1 as a response from llm- 1 indicates positive verification
                if '1' in classification: #user has verified method call
                    self.runner.current_input = "go " + parts[0][0:2] #sets method as runner's current input
                    self.runner.go() #executes method call
                    self.runner.run_loop() #runs loop to execute plan
                    executed = True
                    timer = str(time.perf_counter()-self.startTime)
                    writeToFile("transcript.txt", "**Command executed in " + str(timer) + " seconds**")
                    break #breaks loop because no further fine tuning is needed
                elif i == 1: #user has not verified prompt and all attempts are used
                    outputSpeech("Unable to verify instructions")
                else: #user has not verified prompt, but attempts remain
                    newInfo = getSpeechInput("Please clarify a destination") #gets clarification from user for fine tuning
                    prompt = prompt + newInfo #adds additional instructions to original prompt
                    userRevisions += 1
            elif i<1: #tries again with same prompt because the response from the llm was bad
                if (systemCorrections %2) == 1:
                    newInfo = getSpeechInput("Please clarify an item and destination") #gets clarification from user                  
                    prompt = prompt + newInfo #adds additional instructions to original prompt
                    userRevisions += 1
                else: 
                    outputSpeech("Trying again")
                systemCorrections += 1
            else:
                outputSpeech("Process Failed")
        writeToFile("log.txt", state + "," + str(userRevisions) + "," + str(systemCorrections) + "," + str(executed) + "," + str(timer))
        return RoutingState(self.runner) #returns next state to main method, which is the routing state

class RoutingState(): #state in which the system determines which state the user would like to access
    def __init__(self, runner): #takes runner to initialize- not used directly, but necessary for consitency of state machine and to pass to next state
        self.runner = runner
        #creates instance of llm connector that includes a description of each state and sets the llm up to determine which state the user would like to access
        self.classifier = LLMConnector("phi3:3.8b", """You are part of a larger system that can 
                                       (0) deliver a package
                                       (1) navigate a robot
                                       (2) describe the current state of the system
                                       (3) answer questions regarding the system
                                       Based on the following input, determine which of the abilities the user would like to access and output only the corresponding number, no text. 
                                       """)
    def action(self): #action takes input from user and returns the desired state
        while True: #in loop so that it will try to determine the appropriate state again if the process fails
            input = ""
            while "robot" not in input:
                input = getSpeechInput('')
            request = getSpeechInput("Ready for command") #gives user options and recieves reponse
            startTime = time.perf_counter()
            classification = self.classifier.prompt(request) #prompts llm with user input
            try: 
                num = int(classification) #tries to cast llm response to integer
                if num == 0: #0 means the user wants to deliver a package
                    return PackageDeliveryState(self.runner, request, startTime) #returns next state which is package delivery
                elif num == 1: #1 means the user wants to navigate the robot
                    return NavigationState(self.runner, request, startTime) #returns next state which is navigation
                elif num == 2: #2 means the user want a description of the system
                    return DescriptionState(self.runner, startTime) #returns next state which is description
                elif num == 3: #3 means the user wants to ask a question 
                    return QuestionState(self.runner, request, startTime) #returns next state which is question
                else: outputSpeech("Please try again.")
            except: #catches any errors in case the response is in an improper format
                outputSpeech("Please try again.")

def main(args=None):
    with open(sys.argv[2] + "/" + sys.argv[2] + "_description") as f:
        map_description = f.read()
    with open(sys.argv[2] + "/" + sys.argv[2] + "_map", 'rb') as f:
        map_data = pickle.load(f)
    aliases = dict()
    with open(sys.argv[2] + "/" + sys.argv[2] + "_aliases") as f:
        lines = f.read().splitlines()
        for i in range(0, len(lines)-1, 2):
            aliases[lines[i]] = lines[i+1]
    runner = HTN.RobotMapRunner(sys.argv[2], sys.argv[1], map_data, map_description, aliases)
    runner.st.start()
    runner.ht.start()
    runner.cmd_queue.put('reset')
    systemState = RoutingState(runner) #sets first state of state machine to routing
   # os.system("ollama run phi3:3.8b /bye") #ensures ollama is open locally
    with open(sys.argv[2] + "/" + sys.argv[2] + "_packages") as f:
        for package in f:
            runner.current_input = "at " + package
            runner.at()
    outputSpeech("I am alive")
    while systemState != -1: #continously runs actions of state and gets next state
        systemState = systemState.action()
    runner.finished.set()
    runner.ht.join()
    runner.st.join()
    sys.quit()
if __name__ == '__main__':
    main()


