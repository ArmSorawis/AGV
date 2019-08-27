#!/usr/bin/env python

def readAction(current_station, next_station):

    rotate2currentStation = 0
    rotate2nextStation = 0 

    soundPath = None
    sound_cycleTime = 0

    if current_station == "start":
        soundPath = '/home/agv/agv_ws/src/agv_sound/sound/base2machine.wav' # voice directory
        sound_cycleTime = 4 # second

    elif current_station == "machine":
        rotate2currentStation = 60 # degree

        soundPath = '/home/agv/agv_ws/src/agv_sound/sound/machine.wav' # voice directory
        sound_cycleTime = 4 # second
        
        if next_station == 1:
            rotate2nextStation = 90 # degree
        elif next_station == 5:
            rotate2nextStation = 179 # degree
        elif next_station == 2 or next_station == 3 or next_station == 4:
            rotate2nextStation = -90 # degree

    elif current_station == 1:
        rotate2currentStation = -90 # degree

        soundPath = '/home/agv/agv_ws/src/agv_sound/sound/room1.wav' # voice directory
        sound_cycleTime = 3 # second
        
        if next_station == 'base':
            rotate2nextStation = 90 # degree
        elif next_station == 2 or next_station == 3 or next_station == 4 or next_station == 5:
            rotate2nextStation = -90 # degree

    elif current_station == 2:
        rotate2currentStation = 135 # degree

        soundPath = '/home/agv/agv_ws/src/agv_sound/sound/room2.wav' # voice directory
        sound_cycleTime = 3 # second

        if next_station == 1:
            rotate2nextStation = 90 # degree
        elif next_station == 5:
            rotate2nextStation = 179 # degree
        elif next_station == 3 or next_station == 4:
            rotate2nextStation = -90 # degree
        elif next_station == 'base':
            rotate2nextStation = 90 # degree

    elif current_station == 3:
        rotate2currentStation = 125 # degree

        soundPath = '/home/agv/agv_ws/src/agv_sound/sound/room3.wav' # voice directory
        sound_cycleTime = 3 # second

        if next_station == 4:
            rotate2nextStation = 179 # degree
        elif next_station == 1 or next_station == 2 or next_station == 5:
            rotate2nextStation = 90 # degree
        elif next_station == 'base':
            rotate2nextStation = 90 # degree

    elif current_station == 4:
        rotate2currentStation = 179 # degree

        soundPath = '/home/agv/agv_ws/src/agv_sound/sound/room4.wav' # voice directory
        sound_cycleTime = 3 # second

        if next_station == 3:
            rotate2nextStation = 179 # degree
        elif next_station == 1 or next_station == 2 or next_station == 5:
            rotate2nextStation = -90 # degree
        elif next_station == 'base':
            rotate2nextStation = -90 # degree
    
    elif current_station == 5:
        rotate2currentStation = 179 # degree

        soundPath = '/home/agv/agv_ws/src/agv_sound/sound/room5.wav' # voice directory
        sound_cycleTime = 3 # second

        if next_station == 1:
            rotate2nextStation = -90 # degree
        elif next_station == 2:
            rotate2nextStation = 179 # degree
        elif next_station == 3 or next_station == 4:
            rotate2nextStation = 90 # degree
        elif next_station == 'base':
            rotate2nextStation = -90 # degree

    elif current_station == 'base':
        soundPath = '/home/agv/agv_ws/src/agv_sound/sound/base_station.wav' # voice directory
        sound_cycleTime = 5 # second

        rotate2nextStation = 179 # degree

    readAction.rotate2currentStation = rotate2currentStation
    readAction.rotate2nextStation = rotate2nextStation

    readAction.soundPath = soundPath
    readAction.sound_cycleTime = sound_cycleTime
        
# readAction(1, 'base')
# print(readAction.rotate2nextStation)
