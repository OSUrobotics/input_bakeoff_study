#!/usr/bin/env rosh
from collections import namedtuple
from random import shuffle
import subprocess
import os

DATA_DIR = os.path.join(packages.input_bakeoff_study.path, 'data')

CALIBRATION_TEXT = '<br/><br/>Before continuing,<br/>the mouse input needs to be calibrated.<br/>To continue with calibration, press the spacebar.'
CONTINUE_TEXT = '<br/><br/>If you have any questions, please ask the experimenter now.</br>Press Spacebar when ready to continue.'

def make_participant_id():
    return now().nsecs

Condition = namedtuple('Condition', ['name', 'pre_launches', 'post_launches', 'instructions', 'requires_calibration'])

class Conditions:
    MOUSE = Condition('MOUSE',
                      [],
                      [],
                      'You are playing a bubble popping game.<br/>\
                      The object is to pop as many bubbles as you can in the allotted time.<br/>\
                      For this trial, you will use a regular mouse to pop the bubbles.''',
                      False)
    HEAD  = Condition('HEAD',
                      [packages.head_pose_estimation.launches.estimator_launch,
                       packages.openni_launch.launches.openni_launch,
                       packages.input_bakeoff_study.nodes.face_frame_py],
                       [],
                       'You are playing a bubble popping game.<br/>\
                       The object is to pop as many bubbles as you can in the allotted time.<br/>\
                       For this trial, you will control the mouse with the orientation of your head.<br/>\
                       Do do so, rotate your head left, right, up, and down<br/>to make the cursor move in those directions.<br/>\
                       Press A on the Wiimote to click on a bubble.''' \
                       + CALIBRATION_TEXT,
                       True)
    GLASS = Condition('GLASS',
                      [
                       packages.input_bakeoff_study.launches.glass_frames_launch,
                       packages.input_bakeoff_study.nodes.glassSensorBridge_py,
                       packages.input_bakeoff_study.nodes.start_glass],
                      [packages.input_bakeoff_study.nodes.stop_glass],
                       'You are playing a bubble popping game.<br/>\
                       The object is to pop as many bubbles as you can in the allotted time.<br/>\
                       For this trial, you will control the mouse with the orientation of your head while wearing Google Glass.<br/>\
                       Do do so, rotate your head left, right, up, and down to make the cursor move in those directions.<br/>\
                       Press A on the Wiimote to click on a bubble.<br/><br/>\
                       Please put Glass on now.''' \
                       + CALIBRATION_TEXT,
                       True)
    # EYE   = Condition('EYE',
    #                   [],
    #                   'eye')

    class __metaclass__(type):
        def __iter__(self):
            for attr in dir(Conditions):
                if not attr.startswith('_'):
                    yield getattr(Conditions, attr)

def show_instructions(text, blocking=False):
    splash = packages.input_bakeoff_study.nodes.splashscreen_py('"%s"' % text)
    if blocking:
        sleep(1)
        while splash():
            sleep(0.5)
    else: return splash


particip_id = make_participant_id()

conditions = list(Conditions)
shuffle(conditions)
click_pub = rospy.Publisher('/click', msg.std_msgs.Empty)
for cond in conditions:
    # start up the condition's prerequisites
    launched = [launch(l) for l in cond.pre_launches]

    # put up a splashscreen to give the participant condition-specific instructions
    splash = show_instructions(cond.instructions, blocking=True)

    if cond.requires_calibration:
        launch(packages.input_bakeoff_study.nodes.mouse_node_py)
        show_instructions('Rotate your head so your nose is<br/>pointing at the top left corner of the screen,<br/>then press the spacebar', blocking=True)
        click_pub.publish()
        show_instructions('Rotate your head so your nose is<br/>pointing at the bottom right corner of the screen,<br/>then press the spacebar', blocking=True)
        click_pub.publish()
        show_instructions('Calibration is complete. Press spacebar to continue.', blocking=True)

    # start the game
    output_file_name = os.path.join(DATA_DIR, 'subj_%s_%s' % (particip_id, cond.name))
    game = packages.input_bakeoff_study.nodes.game_sh('%s' % output_file_name)

    # wait until the condition is finished
    sleep(300)

    # end the game (don't worry if this generates a KeyboardInterrupt backgrace)
    kill(game)

    # kill the prereqs
    for l in launched: kill(l)

    # run the postreqs
    post_launched = [launch(l) for l in cond.post_launches]
    sleep(5)
    for l in post_launched: kill(l)