import drRigging.python.components.base as drBase
import drRigging.python.utils.coreUtils as coreUtils
import drRigging.python.utils.curveUtils as curveUtils
import drRigging.python.objects.controls as controls
import pymel.core as pmc
reload(drBase)
reload(controls)
reload(coreUtils)
reload(curveUtils)

class DrTwistySegment(drBase.DrBaseComponent):
    '''
    chain of joints distributed along a curve. with twisting.
    '''
    def __init__(self, start, end, name, numDivisions=4, addMidControls=1, addEndControls=0, cleanUp=1):
        super(DrTwistySegment, self).__init__(name)

        self.build(numDivisions, start, end, addMidControls, addEndControls)

        if cleanUp:
            self.cleanUp()

    def build(self, numDivisions, start, end, addMidControls, addEndControls):
        axisDict = {0:'x', 1:'y', 2:'z'}
        ctrlSize = coreUtils.getDistance(start, end) * .25

        # create inputs - these should be connected to the nodes you wish to drive the twistSegment
        self.startIn = coreUtils.addChild(self.inputGrp, 'group', '%s_start_in_srt' % self.name)
        coreUtils.align(self.startIn, start)
        self.endIn = coreUtils.addChild(self.startIn, 'group', '%s_end_in_srt' % self.name)
        coreUtils.align(self.endIn, end)
        endTranslateList = [abs(attr.get()) for attr in [self.endIn.tx, self.endIn.ty, self.endIn.tz]]
        axis = axisDict[endTranslateList.index(max(endTranslateList))]
        upAxis = 'x'
        if axis in upAxis:
            upAxis = 'y'
        if pmc.getAttr('%s.t%s' % (self.endIn.name(), axis)) < 0.0:
            axis = '-%s' % axis
        self.endIn.setParent(self.inputGrp)

        print 'Axis = %s' % axis


        # build curve
        crv = curveUtils.curveBetweenNodes(start=self.startIn, end=self.endIn, name='%s_crv' % self.name)
        crv.setParent(self.rigGrp)

        # create attributes on main grp
        pmc.addAttr(self.mainGrp, ln='startAuto', at='double', minValue=0, maxValue=1, k=1, h=0)
        pmc.addAttr(self.mainGrp, ln='endAuto', at='double', minValue=0, maxValue=1, k=1, h=0)
        pmc.addAttr(self.mainGrp, ln='twist', at='double')
        pmc.addAttr(self.mainGrp, ln='invertTwist', at=bool, k=0, h=0)

        # world space srt of start and end
        startToSrt = coreUtils.decomposeWorldMatrix(self.startIn)
        endToSrt = coreUtils.decomposeWorldMatrix(self.endIn)

        # Vector representing direction from start to end
        cmpntVec = None
        if '-' in axis:
            cmpntVec = coreUtils.minus([startToSrt.outputTranslate, endToSrt.outputTranslate],'%s_cmpntVector_utl' % self.name)
        else:
            cmpntVec = coreUtils.minus([endToSrt.outputTranslate, startToSrt.outputTranslate], '%s_cmpntVector_utl' % self.name)


        # matrix representing cmpntVec using start as upnode
        cmpntVecNorm = coreUtils.normalizeVector(cmpntVec.output3D, name='%s_cmpntVectorNormalized_utl' % self.name)
        cmpntUpVec = coreUtils.matrixAxisToVector(self.startIn, axis=upAxis, name='%s_cmpntUpVec_utl' % self.name)
        normVec = coreUtils.cross(cmpntVecNorm.output, cmpntUpVec.output, name='%s_localNormalVec_utl' % self.name)

        orderDict = {'x': 0, 'y': 1, 'z': 2, '-x': 0, '-y': 1, '-z': 2}
        matrixList = ['', '', '']
        matrixList[orderDict[axis]] = 'X'
        matrixList[orderDict[upAxis]] = 'Y'

        localOrientMtx = pmc.createNode('fourByFourMatrix', name='%s_localOrientMatrix_utl' % self.name)
        cmpntVecNorm.outputX.connect(pmc.Attribute('%s.in%s0' % (localOrientMtx.name(), matrixList.index('X'))))
        cmpntVecNorm.outputY.connect(pmc.Attribute('%s.in%s1' % (localOrientMtx.name(), matrixList.index('X'))))
        cmpntVecNorm.outputZ.connect(pmc.Attribute('%s.in%s2' % (localOrientMtx.name(), matrixList.index('X'))))

        cmpntUpVec.outputX.connect(pmc.Attribute('%s.in%s0' % (localOrientMtx.name(), matrixList.index('Y'))))
        cmpntUpVec.outputY.connect(pmc.Attribute('%s.in%s1' % (localOrientMtx.name(), matrixList.index('Y'))))
        cmpntUpVec.outputZ.connect(pmc.Attribute('%s.in%s2' % (localOrientMtx.name(), matrixList.index('Y'))))

        normVec.outputX.connect(pmc.Attribute('%s.in%s0' % (localOrientMtx.name(), matrixList.index(''))))
        normVec.outputY.connect(pmc.Attribute('%s.in%s1' % (localOrientMtx.name(), matrixList.index(''))))
        normVec.outputZ.connect(pmc.Attribute('%s.in%s2' % (localOrientMtx.name(), matrixList.index(''))))

        startToSrt.outputTranslateX.connect(localOrientMtx.in30)
        startToSrt.outputTranslateY.connect(localOrientMtx.in31)
        startToSrt.outputTranslateZ.connect(localOrientMtx.in32)


        # matrix representing positional offset of first control
        startEndDist = coreUtils.distanceBetweenNodes(self.startIn, self.endIn, name='%s_startEndDist_utl' % self.name)
        mult = .333
        if '-' in axis:
            mult = -.333
        startLocalOffsetMult = coreUtils.multiply(startEndDist.distance, mult, name='%s_startLocalOffsetMult_utl' % self.name)
        startLocalOffsetMtx = pmc.createNode('fourByFourMatrix', name='%s_startLocalOffsetMtx_utl' % self.name)
        startLocalOffsetMult.outputX.connect('%s.in3%s' % (startLocalOffsetMtx.name(), orderDict[axis[-1]]))

        # Matrices that convert local positional offset into either local orient space or start space
        startLocalMtx = coreUtils.multiplyMatrices([startLocalOffsetMtx.output, localOrientMtx.output],
                                                    name='%s_startLocalMtx_utl' % self.name)
        startLocalMtxToSrt = coreUtils.decomposeMatrix(startLocalMtx.matrixSum, name='%s_startLocalMtxToSrt_utl' % self.name)

        startAutoMtx = coreUtils.multiplyMatrices([startLocalOffsetMtx.output, self.startIn.worldMatrix[0]],
                                                   name='%s_startAutoMtx_utl' % self.name)
        startAutoMtxToSrt = coreUtils.decomposeMatrix(startAutoMtx.matrixSum, name='%s_startAutoMtxToSrt_utl' % self.name)

        startBlend = coreUtils.pairBlend(startLocalMtxToSrt.outputTranslate,
                                         startLocalMtxToSrt.outputRotate,
                                         startAutoMtxToSrt.outputTranslate,
                                         startAutoMtxToSrt.outputRotate,
                                         name = '%s_startAutoBlend_utl' % self.name,
                                         blendAttr=self.mainGrp.startAuto)

        # matrix representing positional offset of second control
        mult = mult *-1
        endLocalOffsetMult = coreUtils.multiply(startEndDist.distance, mult,
                                                  name='%s_endLocalOffsetMult_utl' % self.name)
        endLocalOffsetMtx = pmc.createNode('fourByFourMatrix', name='%s_endLocalOffsetMtx_utl' % self.name)
        endLocalOffsetMult.outputX.connect('%s.in3%s' % (endLocalOffsetMtx.name(), orderDict[axis[-1]]))

        # Matrices that convert local positional offset into either local orient space or start space
        endLocalMtx = coreUtils.multiplyMatrices([endLocalOffsetMtx.output, localOrientMtx.output],
                                                   name='%s_endLocalMtx_utl' % self.name)
        endLocalMtxToSrt = coreUtils.decomposeMatrix(endLocalMtx.matrixSum,
                                                       name='%s_endLocalMtxToSrt_utl' % self.name)

        endLocalPos = pmc.createNode('plusMinusAverage', name='%s_endLocalPos_utl' % self.name)
        if '-' in axis:
            endLocalPos.operation.set(2)
        endLocalMtxToSrt.outputTranslate.connect(endLocalPos.input3D[0])
        cmpntVec.output3D.connect(endLocalPos.input3D[1])

        endAutoMtx = coreUtils.multiplyMatrices([endLocalOffsetMtx.output, self.endIn.worldMatrix[0]],
                                                  name='%s_endAutoMtx_utl' % self.name)
        endAutoMtxToSrt = coreUtils.decomposeMatrix(endAutoMtx.matrixSum,
                                                      name='%s_endAutoMtxToSrt_utl' % self.name)

        endBlend = coreUtils.pairBlend(endLocalPos.output3D,
                                       endLocalMtxToSrt.outputRotate,
                                       endAutoMtxToSrt.outputTranslate,
                                       endAutoMtxToSrt.outputRotate,
                                       name='%s_endAutoBlend_utl' % self.name,
                                       blendAttr=self.mainGrp.endAuto)



        # Motion path
        mps = curveUtils.nodesAlongCurve(crv=crv,
                                         numNodes=numDivisions,
                                         name=self.name,
                                         followAxis=axis,
                                         upAxis=upAxis,
                                         upNode=self.startIn,
                                         upVec=upAxis,
                                         follow=1,
                                         groups=0)

        # twist inversion
        cond = pmc.createNode('condition', name='%s_twistIsInverted_utl' % self.name)
        self.mainGrp.twist.connect(cond.colorIfFalseR)
        uc = coreUtils.convert(self.mainGrp.twist, -1, name='%s_twistInvert_utl' % self.name)
        uc.output.connect(cond.colorIfTrueR)
        self.mainGrp.invertTwist.connect(cond.firstTerm)
        cond.secondTerm.set(1)

        # joints
        for i in range(numDivisions):
            num = str(i + 1).zfill(2)
            mp = mps['mpNodes'][i]
            j = pmc.joint(name='%s_%s_dfm' % (self.name, num))
            j.setParent(self.deformGrp)
            mp.allCoordinates.connect(j.t)
            mp.rotate.connect(j.jointOrient)
            twistBlend = pmc.createNode('animBlendNodeAdditiveRotation', name='%s_%s_twistBlend_utl' % (self.name, num))
            attr = pmc.Attribute('%s.inputA%s' % (twistBlend.name(), axis[-1].upper()))
            cond.outColorR.connect(attr)
            if not '-' in axis:
                twistBlend.weightA.set(mp.uValue.get() * -1)
            else:
                twistBlend.weightA.set(mp.uValue.get())
            twistBlend.output.connect(j.r)

        #twisting
        orientInverseMtx = pmc.createNode('inverseMatrix', name='%s_localOrientInverseMatrix_utl' % self.name)
        localOrientMtx.output.connect(orientInverseMtx.inputMatrix)
        twist = coreUtils.isolateTwist(self.endIn.worldMatrix[0], orientInverseMtx.outputMatrix, name=self.name, axis=axis)
        twistAttr = pmc.Attribute('%s.outputRotate%s' % (twist[2].name(), axis[-1].upper()))
        twistAttr.connect(self.mainGrp.twist)

        crvShape = pmc.listRelatives(crv, c=1, s=1)[0]
        startToSrt.outputTranslate.connect(crvShape.controlPoints[0])
        endToSrt.outputTranslate.connect(crvShape.controlPoints[3])

        # bend controls
        if addMidControls:
            startCtrl = controls.squareCtrl(name='%s_start_ctrl' % self.name, axis=axis[-1], size=ctrlSize)
            startBuffer = coreUtils.addParent(startCtrl, 'group', '%s_startCtrl_buffer_srt' % self.name)
            startBuffer.setParent(self.interfaceGrp)
            startCtrlToSrt = coreUtils.decomposeMatrix(startCtrl.worldMatrix[0], name='%s_startCtrlWorldMtxToSrt_utl' % self.name)
            startBlend.outTranslate.connect(startBuffer.t)
            startBlend.outRotate.connect(startBuffer.r)
            self.ctrls.append(startCtrl)

            endCtrl = controls.squareCtrl(name='%s_end_ctrl' % self.name, axis=axis[-1], size=ctrlSize)
            endBuffer = coreUtils.addParent(endCtrl, 'group', '%s_endCtrl_buffer_srt' % self.name)
            endBuffer.setParent(self.interfaceGrp)
            endCtrlToSrt = coreUtils.decomposeMatrix(endCtrl.worldMatrix[0], name='%s_startendCtrlWorldMtxToSrt_utl' % self.name)
            endBlend.outTranslate.connect(endBuffer.t)
            endBlend.outRotate.connect(endBuffer.r)
            self.ctrls.append(endCtrl)

            startCtrlToSrt.outputTranslate.connect(crvShape.controlPoints[1])
            endCtrlToSrt.outputTranslate.connect(crvShape.controlPoints[2])
        else:
            startBlend.outTranslate.connect(crvShape.controlPoints[1])
            endBlend.outTranslate.connect(crvShape.controlPoints[2])

        if addEndControls:
            self.btmCtrl = controls.circleBumpCtrl(name='%s_btm_ctrl' % self.name, axis=axis, radius=ctrlSize)
            coreUtils.align(self.btmCtrl, self.startIn)
            btmBuffer = coreUtils.addParent(self.btmCtrl, 'group', '%s_btm_buffer_srt' % self.name)
            btmBuffer.setParent(self.interfaceGrp)
            d = coreUtils.isDecomposed(self.btmCtrl)
            coreUtils.connectDecomposedMatrix(d, self.startIn)
            self.ctrls.append(self.btmCtrl)

            self.topCtrl = controls.circleBumpCtrl(name='%s_top_ctrl' % self.name, axis=axis, radius=ctrlSize)
            coreUtils.align(self.topCtrl, self.endIn)
            topBuffer = coreUtils.addParent(self.topCtrl, 'group', '%s_top_buffer_srt' % self.name)
            topBuffer.setParent(self.interfaceGrp)
            d = coreUtils.isDecomposed(self.topCtrl)
            coreUtils.connectDecomposedMatrix(d, self.endIn)
            self.ctrls.append(self.topCtrl)

    def cleanUp(self):
        super(DrTwistySegment, self).cleanUp()
        coreUtils.attrCtrl(nodeList=self.ctrls, attrList=['sx', 'sy', 'sz'])