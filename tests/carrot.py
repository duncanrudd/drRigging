import pymel.core as pmc
import drRigging.utils.coreUtils as coreUtils
reload(coreUtils)

import drRigging.utils.curveUtils as curveUtils
reload(curveUtils)

import drRigging.components.root as drRoot
reload(drRoot)

import drRigging.components.spine as drSpine
reload(drSpine)

import drRigging.components.limb as drLimb
reload(drLimb)

import drRigging.utils.componentUtils as componentUtils
reload(componentUtils)

import drRigging.components.hand as drHand
reload(drHand)

import drRigging.components.reverseFoot as drFoot
reload(drFoot)

import drRigging.components.twistySegment as drTwistySegment
reload(drTwistySegment)

bipedDict = {
    'spine': {'numIntervals': 13, 'numJoints': 7, 'axis': 'y', 'upAxis': 'z', 'ctrlInterval': 3, 'reverse': 1},
    'neck': {'numDivisions': 4},
    'limbs': {'numTwistJoints': 4},
    'hand_L': {'base': [pmc.PyNode('hand_L_hrc_gd')],
              'mid': [pmc.PyNode('hand_L_mid_%s_gd' % str(num+1).zfill(2)) for num in range(5)],
    },
    'hand_R': {'base': [pmc.PyNode('hand_R_hrc_gd')],
              'mid': [pmc.PyNode('hand_R_mid_%s_gd' % str(num+1).zfill(2)) for num in range(5)],
    },
    'foot_L': {'ankle': pmc.PyNode('foot_L_01_gd'),
               'ball': pmc.PyNode('foot_L_02_gd'),
               'toe': pmc.PyNode('foot_L_03_gd'),
               'inner': pmc.PyNode('foot_L_inner_gd'),
               'outer': pmc.PyNode('foot_L_outer_gd'),
               'heel': pmc.PyNode('foot_L_heel_gd'),
    },
    'foot_R': {'ankle': pmc.PyNode('foot_R_01_gd'),
               'ball': pmc.PyNode('foot_R_02_gd'),
               'toe': pmc.PyNode('foot_R_03_gd'),
               'inner': pmc.PyNode('foot_R_inner_gd'),
               'outer': pmc.PyNode('foot_R_outer_gd'),
               'heel': pmc.PyNode('foot_R_heel_gd'),
    },

}


########################################################################################################################
# Build root
root = drRoot.DrRoot('root_M')


########################################################################################################################
# Build body
spineJoints = curveUtils.nodesAlongCurve(numNodes=bipedDict['spine']['numIntervals'],
                                         crv = pmc.PyNode('spine_crv_gd'),
                                         name='spineTemp',
                                         followAxis=bipedDict['spine']['axis'],
                                         upAxis=bipedDict['spine']['upAxis'],
                                         upVec=bipedDict['spine']['upAxis'],
                                         )['grps']

spine = drSpine.DrSpine(name='body_M',
                        joints=spineJoints,
                        numJoints=bipedDict['spine']['numJoints'],
                        axis=bipedDict['spine']['axis'],
                        upAxis=bipedDict['spine']['upAxis'],
                        worldUpAxis=bipedDict['spine']['upAxis'],
                        ctrlInterval=bipedDict['spine']['ctrlInterval'],
                        reverse=bipedDict['spine']['reverse'])
pmc.delete(spineJoints)

# connect spine to root
componentUtils.connectIO(spine.ctrls[0].getParent(), root.ctrls[-1], 'spine_M')


########################################################################################################################
# Shoulders
shldr_R = componentUtils.addCtrl(pmc.PyNode('body_M_04_twist_srt'), pmc.PyNode('body_M_shldr_R_gd'), 'shldr_R', ctrlSize=6)
coreUtils.colorize('red', [shldr_R])

shldr_L = componentUtils.addCtrl(pmc.PyNode('body_M_04_twist_srt'), pmc.PyNode('body_M_shldr_L_gd'), 'shldr_L', ctrlSize=6)
coreUtils.colorize('blue', [shldr_L])

# Add a deformation joint for the shoulders
componentUtils.exposeDeformation(shldr_L, name='shldr_L')
componentUtils.exposeDeformation(shldr_R, name='shldr_R')


########################################################################################################################
'''
# Neck
neck = drTwistySegment.DrTwistySegment(numDivisions=bipedDict['neck']['numDivisions'],
                                         start=pmc.PyNode('neck_M_start_ctrl_gd'),
                                         end=pmc.PyNode('neck_M_head_ctrl_gd'), name='neck_M',
                                         addMidControls=0, addEndControls=1)
coreUtils.colorize('green', neck.ctrls)

# connect neck to body
componentUtils.connectIO(neck.btmCtrl.getParent(), spine.endCtrl, 'body')
neck.mainGrp.startAuto.set(1)

# Space switching for head
targets = [root.ctrls[-1], spine.baseSubCtrl, spine.endCtrl, neck.btmCtrl]
spaces = ['root', 'body', 'chest', 'neck']

componentUtils.addSpaceSwitch(node=neck.topCtrl.getParent(), name='head', spaces=spaces, type='translate',
                              ctrlNode=neck.topCtrl, targetNodes=targets)

componentUtils.addSpaceSwitch(node=neck.topCtrl.getParent(), name='head', spaces=spaces, type='rotate',
                              ctrlNode=neck.topCtrl, targetNodes=targets)

# Add a deformation joint for the head
componentUtils.exposeDeformation(neck.topCtrl, name='neck_M_head')
'''

########################################################################################################################
# Arms
targets = [root.ctrls[-1], spine.baseSubCtrl, pmc.PyNode('body_M_04_twist_srt'), spine.startCtrl]
spaces = ['root', 'body', 'chest', 'hips']

arm_R = drLimb.DrTwistyLimb(name='arm_R',
                            joints=[pmc.PyNode(j) for j in ['arm_R_01_gd', 'arm_R_02_gd', 'arm_R_03_gd', 'arm_R_04_gd']],
                            numTwistJoints=bipedDict['limbs']['numTwistJoints'], alignIkToJointsUpAxis='y')
componentUtils.connectIO(arm_R.baseInput, shldr_R, 'shldr')
componentUtils.addSpaceSwitch(node=arm_R.ikCtrl.getParent(), name='ik', spaces=spaces, type='parent',
                              ctrlNode=arm_R.ikCtrl, targetNodes=targets)
coreUtils.colorize('red', arm_R.ctrls + arm_R.upperTwist.ctrls + arm_R.lowerTwist.ctrls)

arm_L = drLimb.DrTwistyLimb(name='arm_L',
                            joints=[pmc.PyNode(j) for j in ['arm_L_01_gd', 'arm_L_02_gd', 'arm_L_03_gd', 'arm_L_04_gd']],
                            numTwistJoints=bipedDict['limbs']['numTwistJoints'], alignIkToJointsUpAxis='-y')
componentUtils.connectIO(arm_L.baseInput, shldr_L, 'shldr')
componentUtils.addSpaceSwitch(node=arm_L.ikCtrl.getParent(), name='ik', spaces=spaces, type='parent',
                              ctrlNode=arm_L.ikCtrl, targetNodes=targets)
coreUtils.colorize('blue', arm_L.ctrls + arm_L.upperTwist.ctrls + arm_L.lowerTwist.ctrls)


########################################################################################################################
# Hands
hand_L = drHand.DrHand(name='hand_L', fingerDict=bipedDict['hand_L'])
componentUtils.connectIO(hand_L.baseInput, arm_L.resultJoints[-2], 'arm')
coreUtils.colorize('blue', hand_L.ctrls)

hand_R = drHand.DrHand(name='hand_R', fingerDict=bipedDict['hand_R'])
componentUtils.connectIO(hand_R.baseInput, arm_R.resultJoints[-2], 'arm')
coreUtils.colorize('red', hand_R.ctrls)


########################################################################################################################
# Legs
targets = [root.ctrls[-1], spine.baseSubCtrl, spine.startCtrl]
spaces = ['root', 'body', 'hips']

leg_R = drLimb.DrTwistyLimb(name='leg_R',
                            joints=[pmc.PyNode(j) for j in ['leg_R_01_gd', 'leg_R_02_gd', 'leg_R_03_gd', 'leg_R_04_gd']],
                            numTwistJoints=bipedDict['limbs']['numTwistJoints'])
componentUtils.connectIO(leg_R.baseInput, spine.startCtrl, 'hips')
componentUtils.addSpaceSwitch(node=leg_R.ikCtrl.getParent(), name='ik', spaces=spaces, type='parent',
                              ctrlNode=leg_R.ikCtrl, targetNodes=targets)
coreUtils.colorize('red', leg_R.ctrls + leg_R.upperTwist.ctrls + leg_R.lowerTwist.ctrls)

leg_L = drLimb.DrTwistyLimb(name='leg_L',
                            joints=[pmc.PyNode(j) for j in ['leg_L_01_gd', 'leg_L_02_gd', 'leg_L_03_gd', 'leg_L_04_gd']],
                            numTwistJoints=bipedDict['limbs']['numTwistJoints'])
componentUtils.connectIO(leg_L.baseInput, spine.startCtrl, 'hips')
componentUtils.addSpaceSwitch(node=leg_L.ikCtrl.getParent(), name='ik', spaces=spaces, type='parent',
                              ctrlNode=leg_L.ikCtrl, targetNodes=targets)
coreUtils.colorize('blue', leg_L.ctrls + leg_L.upperTwist.ctrls + leg_L.lowerTwist.ctrls)


########################################################################################################################
# Feet
foot_R = drFoot.DrReverseFoot(name='foot_R',
                              ankle=bipedDict['foot_R']['ankle'], ball=bipedDict['foot_R']['ball'],
                              toe=bipedDict['foot_R']['toe'], inner=bipedDict['foot_R']['inner'],
                              outer=bipedDict['foot_R']['outer'], heel=bipedDict['foot_R']['heel'],
                              settingsNode=leg_R.ikCtrl, blendAttr=leg_R.settingsCtrl.ik_fk_blend)
componentUtils.connectIO(foot_R.heelBuffer, leg_R.ikCtrl, 'ik')
componentUtils.connectIO(foot_R.fkBuffer, leg_R.fkCtrls[-1], 'fk')
componentUtils.connectIO(leg_R.ctrl_loc, foot_R.ikBuffer, 'foot')
componentUtils.connectIO(foot_R.ikJoints[0], leg_R.softBlend_loc, 'ik')
coreUtils.colorize('red', foot_R.ctrls)

foot_L = drFoot.DrReverseFoot(name='foot_L',
                              ankle=bipedDict['foot_L']['ankle'], ball=bipedDict['foot_L']['ball'],
                              toe=bipedDict['foot_L']['toe'], inner=bipedDict['foot_L']['inner'],
                              outer=bipedDict['foot_L']['outer'], heel=bipedDict['foot_L']['heel'],
                              settingsNode=leg_L.ikCtrl, blendAttr=leg_L.settingsCtrl.ik_fk_blend)
componentUtils.connectIO(foot_L.heelBuffer, leg_L.ikCtrl, 'ik')
componentUtils.connectIO(foot_L.fkBuffer, leg_L.fkCtrls[-1], 'fk')
componentUtils.connectIO(leg_L.ctrl_loc, foot_L.ikBuffer, 'foot')
componentUtils.connectIO(foot_L.ikJoints[0], leg_L.softBlend_loc, 'ik')
coreUtils.colorize('blue', foot_L.ctrls)


