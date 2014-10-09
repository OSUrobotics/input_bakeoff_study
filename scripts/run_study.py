#!/usr/bin/env rosh
from collections import namedtuple
from random import shuffle
import subprocess
import os

DATA_DIR = os.path.join(packages.input_bakeoff_study.path, 'data')

def make_participant_id():
    return now().nsecs

Condition = namedtuple('Condition', ['name', 'launches', 'instructions'])

class Conditions:
    MOUSE = Condition('MOUSE',
                      [],
                      'You are playing a bubble popping game.<br/>\
                      The object is to pop as many bubbles as you can in the allotted time.<br/>\
                      For this trial, you will use a regular mouse to pop the bubbles''')
    HEAD  = Condition('HEAD',
                      [packages.head_pose_estimation.launches.estimator_launch,
                       packages.openni_launch.launches.openni_launch_launch],
                       'head'
            )
    GLASS = Condition('GLASS',
                      [],
                      'glass')
    EYE   = Condition('EYE',
                      [],
                      'eye')

    class __metaclass__(type):
        def __iter__(self):
            for attr in dir(Conditions):
                if not attr.startswith('_'):
                    yield getattr(Conditions, attr)

def show_instructions(text):
    text += '<br/><br/>Press Spacebar to continue.'
    print text
    return packages.input_bakeoff_study.nodes.splashscreen_py('"%s"' % text)


particip_id = make_participant_id()

conditions = list(Conditions)
shuffle(conditions)
for cond in conditions:
    # start up the condition's prerequisites
    launched = [launch(l) for l in cond.launches]

    # put up a splashscreen to give the participant condition-specific instructions
    splash = show_instructions(cond.instructions)
    sleep(1)
    while splash():
        sleep(0.5)

    # start the game
    output_file_name = os.path.join(DATA_DIR, 'subj_%s_%s' % (particip_id, cond.name))
    game = packages.input_bakeoff_study.nodes.game_sh('%s' % output_file_name)

    # wait until the condition is finished
    sleep(5)

    # end the game (don't worry if this generates a KeyboardInterrupt backgrace)
    kill(game)

    # kill the prereqs
    for l in launched: kill(l)