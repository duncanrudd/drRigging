import drRigging.python.components.base as drBase
reload(drBase)
import drRigging.python.utils.coreUtils as coreUtils
reload(coreUtils)
import drRigging.python.utils.curveUtils as curveUtils
reload(curveUtils)
import drRigging.python.objects.controls as controls
reload(controls)
import pymel.core as pmc
import drRigging.python.utils.componentUtils as componentUtils
reload(componentUtils)

class DrTail(drBase.DrBaseComponent):
    '''
    Builds a tail with two sets of IK controls. The tail joints are distributed along a curve which can be blended
    between either set of controls.
    Each IK control has options to follow root or parent control so inheritance can be broken at any point along the tail.
    If a positive bias value is supplied, divisions along the tail will decrease in length towards the end. A negative
    value will cause the divisions to increase along the tail.
    Tail joints can either stretch full length of curve or maintain default tail length.
    '''
    def __init__(self, start, end, name, numFkCtrls, jointsPerCtrl, numIkCtrls, axis='x', upAxis='y', bias=0, doubleIkChain=1, cleanUp=1):
        super(DrTail, self).__init__(name)

        self.build(start, end, numFkCtrls, jointsPerCtrl, numIkCtrls, axis, upAxis, bias, doubleIkChain)
        
        if cleanUp:
            self.cleanUp()

    def build(self, start, end, numFkCtrls, jointsPerCtrl, numIkCtrls, axis, upAxis, bias, doubleIkChain):
        start, end = coreUtils.getStartAndEnd(start, end)
        ctrlSize = coreUtils.getDistance(start, end) * .2
        # Build ik curves
        ikCrvA = curveUtils.curveBetweenNodes(start, end, numCVs = numIkCtrls, name='%s_ik_A_crv' % self.name)
        ikCrvA.setParent(self.rigGrp)
        if doubleIkChain:
            ikCrvB = curveUtils.curveBetweenNodes(start, end, numCVs=numIkCtrls, name='%s_ik_B_crv' % self.name)
            ikCrvB.setParent(self.rigGrp)

        # main control
        baseCtrl = controls.circleBumpCtrl(radius=ctrlSize,
                                           name='%s_base_ctrl' % self.name, axis=axis)
        baseSrt = coreUtils.decomposeMatrix(baseCtrl.worldMatrix[0], name='%s_baseWorldMtxToSrt_utl' % baseCtrl)
        baseBuffer = coreUtils.addParent(baseCtrl, 'group', name='%s_base_buffer_srt' % self.name)
        baseBuffer.setParent(self.interfaceGrp)
        self.ctrls.append(baseCtrl)

        aimMtx = coreUtils.getAimMatrix(start=start, end=end, axis=axis, upAxis=upAxis, worldUpAxis=upAxis)
        startSrt = coreUtils.matrixToSrt(aimMtx)
        baseBuffer.t.set(startSrt['translate'])
        baseBuffer.r.set(startSrt['rotate'])

        if doubleIkChain:
            pmc.addAttr(baseCtrl, ln='A_B_blend', at='double', k=1, h=0, minValue=0.0, maxValue=1.0)
        pmc.addAttr(baseCtrl, ln='stretch', at='double', k=1, h=0, minValue=0.0, maxValue=1.0)
        pmc.addAttr(baseCtrl, ln='extend', at='double', k=1, h=0, minValue=0.0, maxValue=1.0)
        pmc.addAttr(baseCtrl, ln='twist', at='double', k=1, h=0)
        pmc.addAttr(baseCtrl, ln='upVector', at='enum', enumName='x:y:z', k=1, h=0)
        baseCtrl.extend.set(1)

        # ik ctrls
        cvsA = ikCrvA.getCVs(space='world')
        self.ikACtrls = []
        if doubleIkChain:
            cvsB = ikCrvB.getCVs(space='world')
            self.ikBCtrls = []
        for i in range(numIkCtrls):
            num = str(i).zfill(2)
            if i > 0:
                c = controls.squareCtrl(size = ctrlSize*1.5, name='%s_ik_%s_A_ctrl' % (self.name, num), axis=axis)
                b = coreUtils.addParent(c, 'group', '%s_ik_buffer_A_%s_srt' % (self.name, num))
                b.t.set(cvsA[i])
                b.setParent(self.interfaceGrp)
                d = coreUtils.decomposeMatrix(c.worldMatrix[0], name='%s_ikWorldMtxToSrt_A_%s_utl' % (self.name, num))
                d.outputTranslate.connect(ikCrvA.controlPoints[i])
                componentUtils.addSpaceSwitch(node=b, name='ik_%s' % num, spaces=['base'], type='parent', ctrlNode=c, targetNodes=[baseCtrl])
                self.ctrls.append(c)
                self.ikACtrls.append(c)

                if doubleIkChain:
                    c = controls.squareCtrl(size=ctrlSize * 1.25, name='%s_ik_%s_B_ctrl' % (self.name, num), axis=axis)
                    b = coreUtils.addParent(c, 'group', '%s_ik_buffer_B_%s_srt' % (self.name, num))
                    b.t.set(cvsB[i])
                    b.setParent(self.interfaceGrp)
                    d = coreUtils.decomposeMatrix(c.worldMatrix[0], name='%s_ikWorldMtxToSrt_B_%s_utl' % (self.name, num))
                    d.outputTranslate.connect(ikCrvB.controlPoints[i])
                    componentUtils.addSpaceSwitch(node=b, name='ik_%s' % num, spaces=['base'],
                                                  type='parent', ctrlNode=c, targetNodes=[baseCtrl])

                    self.ikBCtrls.append(c)
                    self.ctrls.append(c)
            else:
                baseSrt.outputTranslate.connect(ikCrvA.controlPoints[0])
                if doubleIkChain:
                    baseSrt.outputTranslate.connect(ikCrvB.controlPoints[0])



        # Arclen stuff for stretching vs maintaining length
        stretchA = curveUtils.getCurveStretch(ikCrvA, name='%s_A' % self.name)
        blendACond = pmc.createNode('condition', name='%s_stretchIsNegativeA_utl' % self.name)
        stretchA['stretchFactor'].outputX.connect(blendACond.firstTerm)
        self.mainGrp.globalScale.connect(stretchA['globalScaleFactor'].input1X)
        #self.mainGrp.globalScale.connect(blendACond.secondTerm)
        blendACond.operation.set(2)
        blendACond.secondTerm.set(1.0)
        blendACond.colorIfTrueR.set(1)
        baseCtrl.stretch.connect(blendACond.colorIfFalseR)
        if doubleIkChain:
            stretchB = curveUtils.getCurveStretch(ikCrvB, name='%s_B' % self.name)
            blendBCond = pmc.createNode('condition', name='%s_stretchIsNegativeB_utl' % self.name)
            stretchB['stretchFactor'].outputX.connect(blendBCond.firstTerm)
            self.mainGrp.globalScale.connect(stretchB['globalScaleFactor'].input1X)
            #self.mainGrp.globalScale.connect(blendBCond.secondTerm)
            blendBCond.secondTerm.set(1.0)
            blendBCond.operation.set(2)
            blendBCond.colorIfTrueR.set(1)
            baseCtrl.stretch.connect(blendBCond.colorIfFalseR)

        # Motion Path nodes
        upVecX = pmc.createNode('colorConstant', name='%s_upVecX_utl' % self.name)
        upVecX.inColor.set((1,0,0))
        upVecY = pmc.createNode('colorConstant', name='%s_upVecY_utl' % self.name)
        upVecY.inColor.set((0, 1, 0))
        upVecZ = pmc.createNode('colorConstant', name='%s_upVecZ_utl' % self.name)
        upVecZ.inColor.set((0, 0, 1))
        upVecChoice = pmc.createNode('choice', name='%s_upVecChoice_utl' % self.name)
        upVecX.outColor.connect(upVecChoice.input[0])
        upVecY.outColor.connect(upVecChoice.input[1])
        upVecZ.outColor.connect(upVecChoice.input[2])

        baseCtrl.upVector.connect(upVecChoice.selector)
        mpsA = curveUtils.nodesAlongCurve(ikCrvA, numNodes=numFkCtrls*jointsPerCtrl,  name='%s_A' % self.name,
                                         followAxis=axis, upAxis=upAxis,
                                         upNode=baseCtrl, follow=1, groups=0)

        if doubleIkChain:
            mpsB = curveUtils.nodesAlongCurve(ikCrvB, numNodes=numFkCtrls*jointsPerCtrl,  name='%s_B' % self.name,
                                             followAxis=axis, upAxis=upAxis,
                                             upNode=baseCtrl, follow=1, groups=0)
        # Parameter values
        paramVals = [mp.uValue.get() for mp in mpsA['mpNodes']]
        if bias:
            for i in range(len(paramVals)):
                paramVals[i] += paramVals[i] - pow(paramVals[i], 1 + bias)

        pairBlends = {'pb':[], 'pbMtx':[], 'pbInverseMtx':[]}

        for i in range(numFkCtrls*jointsPerCtrl):
            num = str(i+1).zfill(2)
            mpA = mpsA['mpNodes'][i]
            baseCtrl.upVector.connect(mpA.upAxis)
            upVecChoice.output.connect(mpA.worldUpVector)
            uc = coreUtils.convert(baseCtrl.twist, ((.0174533 / numFkCtrls*jointsPerCtrl)*i), name='%s_twistMult_%s_utl' % (self.name, num))
            uc.output.connect(mpA.frontTwist)
            paramExtendMult = pmc.createNode('multDoubleLinear', name='%s_paramExtendMult_%s_A_utl' % (self.name, num))
            paramMult = pmc.createNode('multDoubleLinear', name='%s_paramMult_%s_A_utl' % (self.name, num))
            paramExtendMult.input1.set(paramVals[i])
            baseCtrl.extend.connect(paramExtendMult.input2)
            paramExtendMult.output.connect(paramMult.input1)
            stretchA['stretchFactor'].outputX.connect(paramMult.input2)
            blendA = pmc.createNode('blendTwoAttr', name='%s_stretchBlend_%s_A_utl' % (self.name, num))
            paramMult.output.connect(blendA.input[0])
            paramMult.input1.connect(blendA.input[1])

            blendACond.outColorR.connect(blendA.attributesBlender)
            blendA.output.connect(mpA.uValue)

            if doubleIkChain:
                mpB = mpsB['mpNodes'][i]
                baseCtrl.upVector.connect(mpB.upAxis)
                upVecChoice.output.connect(mpB.worldUpVector)
                uc.output.connect(mpB.frontTwist)
                paramMult = pmc.createNode('multDoubleLinear', name='%s_paramMult_%s_B_utl' % (self.name, num))
                paramExtendMult.output.connect(paramMult.input1)
                stretchB['stretchFactor'].outputX.connect(paramMult.input2)
                blendB = pmc.createNode('blendTwoAttr', name='%s_stretchBlend_%s_B_utl' % (self.name, num))
                paramMult.output.connect(blendB.input[0])
                paramMult.input1.connect(blendB.input[1])
                blendBCond.outColorR.connect(blendB.attributesBlender)
                blendB.output.connect(mpB.uValue)

                rv = pmc.createNode('remapValue', name='%s_blendRemap_%s_utl' % (self.name, num))
                baseCtrl.A_B_blend.connect(rv.inputValue)
                rv.value[0].value_Interp.set(2)
                startPos = (1.0 / (numFkCtrls*jointsPerCtrl-1)) * i * .67
                endPos = startPos + .33
                rv.value[0].value_Position.set(startPos)
                rv.value[1].value_Position.set(endPos)

                pb = coreUtils.pairBlend(mpA.allCoordinates, mpA.rotate, mpB.allCoordinates, mpB.rotate,
                                         '%s_curveBlend_%s_utl' % (self.name, num), blendAttr=rv.outValue)
                pbMtx = pmc.createNode('composeMatrix', name='%s_worldMtx_%s_utl' % (self.name, num))
                pb.outTranslate.connect(pbMtx.inputTranslate)
                pb.outRotate.connect(pbMtx.inputRotate)
                baseSrt.outputScale.connect(pbMtx.inputScale)
                pbInverseMtx = pmc.createNode('inverseMatrix', name='%s_worldInverseMtx_%s_utl' % (self.name, num))
                pbMtx.outputMatrix.connect(pbInverseMtx.inputMatrix)

                pairBlends['pb'].append(pb)
                pairBlends['pbMtx'].append(pbMtx)
                pairBlends['pbInverseMtx'].append(pbInverseMtx)
            else:
                mtx = pmc.createNode('composeMatrix', name='%s_worldMtx_%s_utl' % (self.name, num))
                mpA.allCoordinates.connect(mtx.inputTranslate)
                mpA.rotate.connect(mtx.inputRotate)
                baseSrt.outputScale.connect(mtx.inputScale)
                inverseMtx = pmc.createNode('inverseMatrix', name='%s_worldInverseMtx_%s_utl' % (self.name, num))
                mtx.outputMatrix.connect(inverseMtx.inputMatrix)

                pairBlends['pbMtx'].append(mtx)
                pairBlends['pbInverseMtx'].append(inverseMtx)

        # Build matrix chain and fk controls
        self.fkCtrls = []
        buffers = []
        d=None
        srtList=[]

        for i in range(numFkCtrls*jointsPerCtrl):
            num = str(i + 1).zfill(2)
            ctrlNum = str((i / jointsPerCtrl) + 1).zfill(2)
            b = coreUtils.addChild(self.interfaceGrp, 'group', '%s_fk_buffer_%s_srt' % (self.name, num))
            buffers.append(b)
            if i % jointsPerCtrl == 0:
                c = controls.circleBumpCtrl(name='%s_fk_%s_ctrl' % (self.name, ctrlNum),
                                        radius=ctrlSize * .5, axis=axis)

                c.setParent(b)
                self.fkCtrls.append(c)
                self.ctrls.append(c)
            else:
                b.setParent(self.rigGrp)

            if i != 0:
                multMtx = None
                ctrl = self.fkCtrls[-1]
                if i % jointsPerCtrl == 0:
                    ctrl = self.fkCtrls[-2]
                multMtx = coreUtils.multiplyMatrices([pairBlends['pbMtx'][i].outputMatrix,
                                                      pairBlends['pbInverseMtx'][i-1].outputMatrix,
                                                      ctrl.matrix,
                                                      buffers[-2].worldMatrix[0]],
                                                     name='%s_localIkToFkMtx_%s_utl' % (self.name, num))

                d = coreUtils.decomposeMatrix(multMtx.matrixSum, name='%s_ikToFkMtxToSrt_%s_utl' % (self.name, num))
            else:
                d = coreUtils.decomposeMatrix(pairBlends['pbMtx'][0].outputMatrix,
                                                  name='%s_ikToFkMtxToSrt_%s_utl' % (self.name, num))
            d.outputTranslate.connect(b.t)
            d.outputRotate.connect(b.r)
            d.outputScale.connect(b.s)
            srtList.append(d)


        # Create joints
        for i in range(numFkCtrls*jointsPerCtrl):
            num = str(i + 1).zfill(2)
            j = coreUtils.addChild(self.deformGrp, 'joint', '%s_%s_dfm' % (self.name, num))
            srtList[i].outputTranslate.connect(j.t)
            srtList[i].outputRotate.connect(j.r)
            srtList[i].outputScale.connect(j.s)
            
    def cleanUp(self):
        super(DrTail, self).cleanUp()
        coreUtils.attrCtrl(nodeList=self.fkCtrls, attrList=['tx', 'ty', 'tz', 'sx', 'sy', 'sz'])
        coreUtils.attrCtrl(nodeList=self.ikACtrls, attrList=['sx', 'sy', 'sz', 'rx', 'ry', 'rz'])

