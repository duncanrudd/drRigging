import pymel.core as pmc
import drRigging.utils.coreUtils as coreUtils
import drRigging.utils.curveUtils as curveUtils
import drRigging.objects.controls as controls
import drRigging.utils.componentUtils as componentUtils
import drRigging.components.base as drBase
reload(componentUtils)
reload(drBase)
reload(curveUtils)
reload(coreUtils)
reload(controls)

class DrHand(drBase.DrBaseComponent):
    '''
    builds an fk hand based on fingerDict - a dictionary which should contain a key for 'base' (the origin of the hand)
    should also contain a key for each finger ('pinky', 'index' etc)
    values for each finger key should be a list of joints which will be used to build the rig for that finger.
    exanple fingerDict:
    {
    'base':'arm_L_wrist_srt'
    'index':['joint1', 'joint2', joint3', 'joint4']
    'mid':['joint5', 'joint6', joint7', 'joint8']
    'pinky':['joint9', 'joint10', joint11', 'joint12']
    }
    '''
    def __init__(self, name, fingerDict, addTranslateJoints=1, cleanUp=1):
        super(DrHand, self).__init__(name=name)

        self.build(fingerDict, addTranslateJoints)

        if cleanUp:
            self.cleanUp()

    def build(self, fingerDict, addTranslateJoints):
        self.baseInput = coreUtils.addChild(self.inputGrp, 'group', '%s_base_in_srt' % self.name)
        coreUtils.align(self.baseInput, fingerDict['base'])

        ctrlsGrp = coreUtils.addChild(self.interfaceGrp, 'group', '%s_ctrls_srt' % self.name)
        d = coreUtils.isDecomposed(self.baseInput)
        coreUtils.connectDecomposedMatrix(d, ctrlsGrp)

        deformGrp = coreUtils.addChild(self.deformGrp, 'group', '%s_deform_srt' % self.name)
        d = coreUtils.isDecomposed(self.baseInput)
        coreUtils.connectDecomposedMatrix(d, deformGrp)

        ctrlSize = None


        fingerKeys = [k for k in fingerDict.keys() if not k == 'base']
        for finger in fingerKeys:
            joints = []
            fingerList = fingerDict[finger]
            axis = coreUtils.getDominantAxis(fingerList[0], fingerList[1])
            if not ctrlSize:
                ctrlSize = coreUtils.getDistance(fingerList[0], fingerList[1]) * .25
            for i in range(len(fingerList)):
                guide = fingerList[i]
                num = str(i+1).zfill(2)
                c = controls.circleBumpCtrl(radius=ctrlSize, axis=axis, name='%s_%s_%s_ctrl' % (self.name, finger, num))
                coreUtils.align(c, guide)
                buffer = coreUtils.addParent(c, 'group', '%s_%s_%s_buffer_srt' % (self.name, finger, num))
                if i == 0:
                    buffer.setParent(ctrlsGrp)
                else:
                    buffer.setParent(self.ctrls[-1])
                self.ctrls.append(c)

                if addTranslateJoints:

                    b = coreUtils.addChild(deformGrp, 'group', '%s_%s_%s_dfmBuffer_srt' % (self.name, finger, num))
                    coreUtils.align(b, buffer)
                    jt = coreUtils.addChild(b, 'joint', '%s_%s_%s_translate_dfm' % (self.name, finger, num))
                    jr = coreUtils.addChild(jt, 'joint', '%s_%s_%s_rotate_dfm' % (self.name, finger, num))
                    self.ctrls[-1].t.connect(jt.t)
                    self.ctrls[-1].r.connect(jr.r)
                    self.ctrls[-1].s.connect(jr.s)

                    if i != 0:
                        b.setParent(joints[-1])
                    joints.append(jr)

                    pmc.setAttr('%s.type' % jt, 18)
                    jt.otherType.set('translateJoint')
                    pmc.setAttr('%s.type' % jr, 18)
                    jr.otherType.set('rotateJoint')
                else:
                    b = coreUtils.addChild(deformGrp, 'group', '%s_%s_%s_dfmBuffer_srt' % (self.name, finger, num))
                    coreUtils.align(b, buffer)
                    j = coreUtils.addChild(b, 'joint', '%s_%s_%s_dfm' % (self.name, finger, num))
                    self.ctrls[-1].t.connect(j.t)
                    self.ctrls[-1].r.connect(j.r)
                    self.ctrls[-1].s.connect(j.s)
                    if i != 0:
                        b.setParent(joints[-1])
                    joints.append(j)

    def cleanUp(self):
        super(DrHand, self).cleanUp()






