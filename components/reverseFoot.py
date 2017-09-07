import pymel.core as pmc
import drRigging.python.utils.coreUtils as coreUtils
import drRigging.python.utils.curveUtils as curveUtils
import drRigging.python.objects.controls as controls
import drRigging.python.utils.componentUtils as componentUtils
import drRigging.python.components.base as drBase
reload(componentUtils)
reload(drBase)
reload(curveUtils)
reload(coreUtils)
reload(controls)

class DrReverseFoot(drBase.DrBaseComponent):
    def __init__(self, name, ankle, ball, toe, inner, outer, heel, settingsNode=None, blendAttr=None, cleanUp=1):
        '''
        Builds a reverse ik fk foot setup with attributes to manipulate the ik structure and fk controls to manipulate
        the fk structure.

        Currently this component is predicated on the assumption that x will be the twisting axis, y is the rolling axis
        and z is the leaning axis.

        :param name:
        :param ankle: joint representing start of chains
        :param ball: joint representing middle of chains
        :param toe: joint representing end of chains
        :param inner: transform representing inside edge of foot
        :param outer: transform representing outside edge of foot
        :param heel: transform representing location of heel
        :param settingsNode: node to add control attribute to
        :param blendAttr: attribute that controls blending between ik and fk
        :param cleanUp:
        '''
        super(DrReverseFoot, self).__init__(name=name)

        self.build(ankle, ball, toe, inner, outer, heel, settingsNode, blendAttr)

        if cleanUp:
            self.cleanUp()

    def build(self, ankle, ball, toe, inner, outer, heel, settingsNode=None, blendAttr=None):
        # duplicate joints
        self.resultJoints = coreUtils.duplicateJoints([ankle, ball, toe], name='%s_result_XX_jnt' % self.name)
        self.fkJoints = coreUtils.duplicateJoints([ankle, ball, toe], name='%s_fk_XX_jnt' % self.name)
        self.ikJoints = coreUtils.duplicateJoints([ankle, ball, toe], name='%s_ik_XX_jnt' % self.name)

        self.resultBuffer = coreUtils.addChild(self.rigGrp, 'group', '%s_resultJoints_buffer_srt' % self.name)
        coreUtils.align(self.resultBuffer, self.resultJoints[0])
        self.resultJoints[0].setParent(self.resultBuffer)

        self.fkBuffer = coreUtils.addChild(self.rigGrp, 'group', '%s_fkJoints_buffer_srt' % self.name)
        coreUtils.align(self.fkBuffer, self.resultJoints[0])
        self.fkJoints[0].setParent(self.fkBuffer)

        self.ikBuffer = coreUtils.addChild(self.rigGrp, 'group', '%s_ikJoints_buffer_srt' % self.name)
        coreUtils.align(self.ikBuffer, self.resultJoints[0])
        self.ikJoints[0].setParent(self.ikBuffer)

        # set up blending
        if not blendAttr:
            pmc.addAttr(self.mainGrp, ln='ik_fk_blend', at='double', minValue=0, maxValue=1, k=1, h=0)
            blendAttr = self.mainGrp.ik_fk_blend
        for rj, ij, fj in zip([self.resultBuffer] + self.resultJoints, [self.ikBuffer] + self.ikJoints, [self.fkBuffer] + self.fkJoints):
            coreUtils.ikFkBlend(ij, fj, rj, blendAttr=blendAttr)

        ctrlSize = coreUtils.getDistance(self.resultJoints[0], self.resultJoints[1]) * .75

        # Orientation for controls and aim constraints
        axis = 'x'
        aimVec = (1, 0, 0)
        if self.resultJoints[1].tx.get() < 0.0:
            axis = '-x'
            aimVec = (-1, 0, 0)

        # FK ctrl
        self.toeCtrl = controls.circleBumpCtrl(axis='x', radius=ctrlSize, name='%s_fkToe_ctrl')
        self.toeCtrlBuffer = coreUtils.addParent(self.toeCtrl, 'group', name='%s_fkToe_buffer_srt')
        coreUtils.align(self.toeCtrlBuffer, ball)
        self.toeCtrlBuffer.setParent(self.interfaceGrp)
        self.toeCtrl.r.connect(self.fkJoints[1].r)
        self.ctrls.append(self.toeCtrl)
        blendAttr.connect(self.toeCtrlBuffer.visibility)

        # ik Driving structure
        self.heel = coreUtils.createAlignedNode(heel, 'group', '%s_heel_srt' % self.name)
        self.heel.setParent(self.rigGrp)
        self.heelBuffer = coreUtils.addParent(self.heel, 'group', '%s_heel_buffer_srt' % self.name)

        self.toe = coreUtils.createAlignedNode(toe, 'group', '%s_toe_srt' % self.name)
        self.toe.setParent(self.heel)

        self.inner = coreUtils.createAlignedNode(inner, 'group', '%s_inner_srt' % self.name)
        self.inner.setParent(self.toe)
        innerBuffer = coreUtils.addParent(self.inner, 'group', '%s_inner_buffer_srt' % self.name)

        self.outer = coreUtils.createAlignedNode(outer, 'group', '%s_outer_srt' % self.name)
        self.outer.setParent(self.inner)
        outerBuffer = coreUtils.addParent(self.outer, 'group', '%s_outer_buffer_srt' % self.name)

        self.ballPivot = coreUtils.createAlignedNode(ball, 'group', '%s_ballPivot_srt' % self.name)
        self.ballPivot.setParent(self.outer)
        coreUtils.addParent(self.ballPivot, 'group', '%s_ballPivot_buffer_srt' % self.name)

        pmc.transformLimits(self.inner, rx=(0, 0), erx=(1, 0))

        pmc.transformLimits(self.outer, rx=(0, 0), erx=(0, 1))

        self.ball = coreUtils.createAlignedNode(ball, 'group', '%s_ball_srt' % self.name)
        self.ball.setParent(self.ballPivot)
        self.ballBuffer = coreUtils.addParent(self.ball, 'group', '%s_ball_buffer_srt' % self.name)
        self.toeWiggle = coreUtils.addChild(self.ballBuffer, 'group', '%s_toeWiggle_srt' % self.name)

        componentUtils.connectIO(self.ikBuffer, self.ball, 'ball')

        # ikHandles
        ballIkHandle = pmc.ikHandle(solver='ikRPsolver',
                                    name='%s_ball_ikHandle' % self.name,
                                    startJoint=self.ikJoints[0],
                                    endEffector=self.ikJoints[1],
                                    setupForRPsolver=1)[0]
        ballIkHandle.setParent(self.ball)

        toeIkHandle = pmc.ikHandle(solver='ikRPsolver',
                                   name='%s_toe_ikHandle' % self.name,
                                   startJoint=self.ikJoints[1],
                                   endEffector=self.ikJoints[2],
                                   setupForRPsolver=1)[0]
        toeIkHandle.setParent(self.toeWiggle)

        # IK driving attributes
        if not settingsNode:
            settingsNode = self.mainGrp

        attrList = ['heel_roll', 'heel_pivot',
                    'ball_roll', 'ball_pivot', 'ball_twist', 'ball_bend',
                    'toe_roll', 'toe_pivot', 'toe_twist', 'toe_wiggle', 'toe_bend',
                    'edge_roll']

        pmc.addAttr(settingsNode, ln='ik_foot_attributes', k=0, h=0, at='enum', enumName=' ')
        pmc.setAttr(settingsNode.ik_foot_attributes, cb=1, lock=1)

        for attr in attrList:
            pmc.addAttr(settingsNode, ln=attr, at='double', k=1, h=0)

        settingsNode.heel_roll.connect(self.heel.ry)
        settingsNode.heel_pivot.connect(self.heel.rz)
        settingsNode.ball_roll.connect(self.ball.ry)
        settingsNode.ball_pivot.connect(self.ballPivot.rz)
        settingsNode.ball_twist.connect(self.ball.rx)
        settingsNode.ball_bend.connect(self.ball.rz)
        settingsNode.toe_roll.connect(self.toe.ry)
        settingsNode.toe_pivot.connect(self.toe.rz)
        settingsNode.toe_twist.connect(self.toeWiggle.rx)
        settingsNode.toe_wiggle.connect(self.toeWiggle.ry)
        settingsNode.toe_bend.connect(self.toeWiggle.rz)
        settingsNode.edge_roll.connect(self.inner.rx)
        settingsNode.edge_roll.connect(self.outer.rx)

        # Deform joints
        self.dfmJoints = coreUtils.duplicateJoints([ankle, ball, toe], name='%s_XX_dfm' % self.name)
        self.dfmBufferGrp = coreUtils.addChild(self.deformGrp, 'group', '%s_dfmJoints_buffer_srt' % self.name)
        coreUtils.align(self.dfmBufferGrp, self.resultJoints[0])
        self.resultBuffer.t.connect(self.dfmBufferGrp.t)
        self.resultBuffer.r.connect(self.dfmBufferGrp.r)
        self.resultBuffer.s.connect(self.dfmBufferGrp.s)

        self.dfmJoints[0].setParent(self.dfmBufferGrp)
        for i in range(3):
            self.resultJoints[i].t.connect(self.dfmJoints[i].t)
            self.resultJoints[i].r.connect(self.dfmJoints[i].r)
            self.resultJoints[i].s.connect(self.dfmJoints[i].s)


    def cleanUp(self):
        super(DrReverseFoot, self).cleanUp()
        coreUtils.attrCtrl(nodeList=[self.toeCtrl], attrList=['tx', 'ty', 'tz', 'sx', 'sy', 'sz'])
