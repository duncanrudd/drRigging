import drRigging.components.base as drBase

reload(drBase)
import drRigging.utils.coreUtils as coreUtils

reload(coreUtils)
import drRigging.utils.curveUtils as curveUtils

reload(curveUtils)
import drRigging.objects.controls as controls

reload(controls)
import pymel.core as pmc
import drRigging.utils.componentUtils as componentUtils

reload(componentUtils)


class DrVine(drBase.DrBaseComponent):
    '''
    Builds an ik curve. This curve is divided into a given number of fk chain 'segments' which can 'grow' along the curve.
    Upvectors are progressively sampled along the curve based on the previous node in the chain to avoid flipping when
    curvature becomes tight. 
    
    '''

    def __init__(self, start, end, name, numFkCtrls, jointsPerCtrl, numIkCtrls, numSegments,
                 axis='x', upAxis='y', bias=0, cleanUp=1):
        super(DrVine, self).__init__(name)

        self.build(start, end, numFkCtrls, jointsPerCtrl, numIkCtrls, numSegments, axis, upAxis, bias)

        if cleanUp:
            self.cleanUp()

    def build(self, start, end, numFkCtrls, jointsPerCtrl, numIkCtrls, numSegments, axis, upAxis, bias):
        start, end = coreUtils.getStartAndEnd(start, end)
        ctrlSize = coreUtils.getDistance(start, end) * .3 / numSegments
        # Build ik curves
        ikCrv = curveUtils.curveBetweenNodes(start, end, numCVs=numIkCtrls, name='%s_ik_crv' % self.name)
        ikCrv.setParent(self.rigGrp)

        # main control
        baseCtrl = controls.circleBumpCtrl(radius=ctrlSize*1.25,
                                           name='%s_base_ctrl' % self.name, axis=axis)
        baseSrt = coreUtils.decomposeMatrix(baseCtrl.worldMatrix[0], name='%s_baseWorldMtxToSrt_utl' % baseCtrl)
        baseBuffer = coreUtils.addParent(baseCtrl, 'group', name='%s_base_buffer_srt' % self.name)
        baseBuffer.setParent(self.interfaceGrp)

        pmc.addAttr(baseCtrl, ln='stretch', at='double', k=1, h=0, minValue=0.0, maxValue=1.0)

        self.ctrls.append(baseCtrl)

        aimMtx = coreUtils.getAimMatrix(start=start, end=end, axis=axis, upAxis=upAxis, worldUpAxis=upAxis)
        startSrt = coreUtils.matrixToSrt(aimMtx)
        baseBuffer.t.set(startSrt['translate'])
        baseBuffer.r.set(startSrt['rotate'])

        # ik ctrls
        cvs = ikCrv.getCVs(space='world')
        self.ikCtrls = []
        
        for i in range(numIkCtrls):
            num = str(i).zfill(2)
            if i > 0:
                c = controls.ballCtrl(radius=ctrlSize - (i*.25), name='%s_ik_%s_ctrl' % (self.name, num))
                b = coreUtils.addParent(c, 'group', '%s_ik_buffer_%s_srt' % (self.name, num))
                b.t.set(cvs[i])
                b.setParent(self.interfaceGrp)
                d = coreUtils.decomposeMatrix(c.worldMatrix[0], name='%s_ikWorldMtxToSrt_%s_utl' % (self.name, num))
                d.outputTranslate.connect(ikCrv.controlPoints[i])
                componentUtils.addSpaceSwitch(node=b, name='ik_%s' % num, spaces=['base'], type='parent', ctrlNode=c,
                                              targetNodes=[baseCtrl])
                self.ctrls.append(c)
                self.ikCtrls.append(c)
            else:
                baseSrt.outputTranslate.connect(ikCrv.controlPoints[0])

        # Arclen stuff for stretching vs maintaining length
        stretch = curveUtils.getCurveStretch(ikCrv, name='%s_A' % self.name)
        blendCond = pmc.createNode('condition', name='%s_stretchIsNegative_utl' % self.name)
        stretch['stretchFactor'].outputX.connect(blendCond.firstTerm)
        self.mainGrp.globalScale.connect(stretch['globalScaleFactor'].input1X)
        # self.mainGrp.globalScale.connect(blendCond.secondTerm)
        blendCond.operation.set(2)
        blendCond.secondTerm.set(1.0)
        blendCond.colorIfTrueR.set(1)
        baseCtrl.stretch.connect(blendCond.colorIfFalseR)

        mpList = []
        settingsCtrls = []
        allBuffers = []
        upNode = baseCtrl
        # Motion Path nodes
        for x in range(numSegments):
            segNum = str(x + 1).zfill(2)

            settingsCtrl = controls.squareChamferCtrl(size=ctrlSize - (x*.1), axis=upAxis, name='%s_seg_%s_settings_ctrl' % (self.name, segNum))
            pmc.addAttr(settingsCtrl, ln='extend', at='double', k=1, h=0, minValue=0.0, maxValue=1.0)
            pmc.addAttr(settingsCtrl, ln='twist', at='double', k=1, h=0)
            pmc.addAttr(settingsCtrl, ln='width', at='double', k=1, h=0)
            pmc.addAttr(settingsCtrl, ln='taper', at='double', k=1, h=0)
            settingsCtrls.append(settingsCtrl)
            settingsCtrl.extend.set(1)

            if x != 0:
                upNode = buffers[-1]
            minRange = (1.0 / numSegments) * x
            maxRange = (1.0 / numSegments) * (x + 1)
            mpList.append(curveUtils.nodesAlongCurve(ikCrv, numNodes=numFkCtrls * jointsPerCtrl,
                                                     name='%s_%s' % (self.name, segNum), followAxis=axis, upAxis=upAxis,
                                                     upNode=upNode, follow=1, groups=0, progressive=1,
                                                     sampleRange=(minRange, maxRange)))

            # Parameter values
            paramVals = [mp.uValue.get() for mp in mpList[-1]['mpNodes']]
            if bias:
                for x in range(len(paramVals)):
                    paramVals[x] += paramVals[x] - pow(paramVals[x], 1 + bias)

            pairBlends = {'pb': [], 'pbMtx': [], 'pbInverseMtx': []}

            for i in range(numFkCtrls * jointsPerCtrl):
                num = str(i + 1).zfill(2)
                mp = mpList[-1]['mpNodes'][i]
                uc = coreUtils.convert(settingsCtrl.twist, ((.0174533 / numFkCtrls * jointsPerCtrl) * i),
                                       name='%s_seg_%s_twistMult_%s_utl' % (self.name, segNum, num))
                uc.output.connect(mp.frontTwist)

                if i != 0:
                    paramExtendMult = pmc.createNode('multDoubleLinear',
                                                     name='%s_seg_%s_paramExtendMult_%s_utl' % (self.name, segNum, num))
                    paramMult = pmc.createNode('multDoubleLinear',
                                               name='%s_seg_%s_paramMult_%s_utl' % (self.name, segNum, num))
                    if x != 0:
                        paramExtendMult.input1.set(paramVals[i] - minRange)
                    else:
                        paramExtendMult.input1.set(paramVals[i])
                    settingsCtrl.extend.connect(paramExtendMult.input2)

                    paramExtendMult.output.connect(paramMult.input1)
                    stretch['stretchFactor'].outputX.connect(paramMult.input2)
                    #paramMult.input2.set(1)

                    blend = pmc.createNode('blendTwoAttr', name='%s_seg_%s_stretchBlend_%s_utl' % (self.name, segNum, num))
                    if x != 0:
                        paramExtendOffset = coreUtils.add([paramMult.output, mpList[-1]['mpNodes'][0].uValue],
                                                          name='%s_seg_%s_paramExtendOffset_%s_utl' % (self.name, segNum, num))
                        paramExtendOffset.output1D.connect(blend.input[0])

                        paramMultOffset = coreUtils.add([paramMult.input1, mpList[-1]['mpNodes'][0].uValue],
                                                          name='%s_seg_%s_paramMultOffset_%s_utl' % (
                                                          self.name, segNum, num))
                        paramMultOffset.output1D.connect(blend.input[1])
                    else:
                        paramMult.output.connect(blend.input[0])
                        paramMult.input1.connect(blend.input[1])

                    blendCond.outColorR.connect(blend.attributesBlender)
                    blend.output.connect(mp.uValue)
                else:
                    if x != 0:
                        mpList[-2]['mpNodes'][-1].uValue.connect(mp.uValue)

                mtx = pmc.createNode('composeMatrix', name='%s_seg_%s_worldMtx_%s_utl' % (self.name, segNum, num))
                mp.allCoordinates.connect(mtx.inputTranslate)
                mp.rotate.connect(mtx.inputRotate)
                baseSrt.outputScale.connect(mtx.inputScale)
                inverseMtx = pmc.createNode('inverseMatrix', name='%s_seg_%s_worldInverseMtx_%s_utl' % (self.name, segNum, num))
                mtx.outputMatrix.connect(inverseMtx.inputMatrix)

                pairBlends['pbMtx'].append(mtx)
                pairBlends['pbInverseMtx'].append(inverseMtx)

            # Build matrix chain and fk controls
            self.fkCtrls = []
            buffers = []
            d = None
            srtList = []

            for i in range(numFkCtrls * jointsPerCtrl):
                num = str(i + 1).zfill(2)
                ctrlNum = str((i / jointsPerCtrl) + 1).zfill(2)
                b = coreUtils.addChild(self.interfaceGrp, 'group', '%s_seg_%s_fk_buffer_%s_srt' % (self.name, segNum, num))
                buffers.append(b)
                allBuffers.append(b)
                if i % jointsPerCtrl == 0:
                    c = controls.circleBumpCtrl(name='%s_seg_%s_fk_%s_ctrl' % (self.name, segNum, ctrlNum),
                                                radius=ctrlSize * .5 - (i*.02) - (x*.1), axis=axis)

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
                                                          pairBlends['pbInverseMtx'][i - 1].outputMatrix,
                                                          ctrl.matrix,
                                                          buffers[-2].worldMatrix[0]],
                                                         name='%s_localIkToFkMtx_%s_utl' % (self.name, num))

                    d = coreUtils.decomposeMatrix(multMtx.matrixSum, name='%s_seg_%s_ikToFkMtxToSrt_%s_utl' % (self.name, segNum, num))
                else:
                    if x == 0:
                        d = coreUtils.decomposeMatrix(pairBlends['pbMtx'][0].outputMatrix,
                                                      name='%s_seg_%s_ikToFkMtxToSrt_%s_utl' % (self.name, segNum, num))
                    else:
                        d = coreUtils.decomposeMatrix(allBuffers[-2].worldMatrix[0], name='%s_seg_%s_ikToFkMtxToSrt_%s_utl' % (self.name, segNum, num))
                    settingsCtrl.setParent(b)
                d.outputTranslate.connect(b.t)
                d.outputRotate.connect(b.r)
                d.outputScale.connect(b.s)
                srtList.append(d)

            widthOffset = coreUtils.add([settingsCtrl.width, settingsCtrl.taper, 1], name='%s_%s_widthOffset_utl' % (self.name, segNum))
            widthMult = pmc.createNode('multDoubleLinear', name='%s_%s_widthMult_utl' % (self.name, segNum))
            widthOffset.output1D.connect(widthMult.input1)
            baseSrt.outputScaleX.connect(widthMult.input2)

            # Create joints
            for i in range(numFkCtrls * jointsPerCtrl):
                num = str(i + 1).zfill(2)
                j = coreUtils.addChild(self.deformGrp, 'joint', '%s_seg_%s_%s_dfm' % (self.name, segNum, num))
                srtList[i].outputTranslate.connect(j.t)
                srtList[i].outputRotate.connect(j.r)
                taperMult = coreUtils.convert(settingsCtrl.taper, i*.1, name='%s_%s_taperMult_%s_utl' % (self.name, segNum, num))
                taperAmount = coreUtils.add([widthOffset.output1D, taperMult.output], name='%s_%s_taperAmount_%s_utl' % (self.name, segNum, num))
                taperAmount.output1D.connect(j.sx)
                taperAmount.output1D.connect(j.sy)
                taperAmount.output1D.connect(j.sz)
                scaleAttr = pmc.Attribute('%s.s%s' % (j.name(), axis[-1]))
                srtList[i].outputScaleX.connect(scaleAttr, f=1)

    def cleanUp(self):
        super(DrVine, self).cleanUp()
        coreUtils.attrCtrl(nodeList=self.fkCtrls, attrList=['tx', 'ty', 'tz', 'sx', 'sy', 'sz'])
        coreUtils.attrCtrl(nodeList=self.ikCtrls, attrList=['sx', 'sy', 'sz', 'rx', 'ry', 'rz'])

