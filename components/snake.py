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

import math

class DrSnake(drBase.DrBaseComponent):
    '''

    '''
    def __init__(self, start, end, name, numFkCtrls, jointsPerCtrl, numIkCtrls, latticeDivisions=4, axis='x', upAxis='y',  cleanUp=1):
        super(DrSnake, self).__init__(name)

        self.build(start, end, numFkCtrls, jointsPerCtrl, numIkCtrls, latticeDivisions, axis, upAxis)

        if cleanUp:
            self.cleanUp()

    def build(self, start, end, numFkCtrls, jointsPerCtrl, numIkCtrls, latticeDivisions, axis, upAxis):
        start, end = coreUtils.getStartAndEnd(start, end)
        ctrlSize = coreUtils.getDistance(start, end) * .1
        radius=1

        # main control
        baseCtrl = controls.circleBumpCtrl(radius=ctrlSize,
                                           name='%s_base_ctrl' % self.name, axis=axis)
        baseSrt = coreUtils.decomposeMatrix(baseCtrl.worldMatrix[0], name='%s_baseWorldMtxToSrt_utl' % baseCtrl)
        baseBuffer = coreUtils.addParent(baseCtrl, 'group', name='%s_base_buffer_srt' % self.name)
        baseBuffer.setParent(self.interfaceGrp)
        self.ctrls.append(baseCtrl)

        points = coreUtils.pointsAlongVector(start, end, divisions=numIkCtrls-1)
        points.insert(1, coreUtils.pointsAlongVector(points[0], points[1], 2)[1])
        points.insert(numIkCtrls, coreUtils.pointsAlongVector(points[-2], points[-1], 2)[1])

        # ik ctrls
        ikCtrls=[]
        for i in range(len(points)):
            num = str(i+1).zfill(2)

            c = controls.ballCtrl(radius=ctrlSize*.5, name='%s_ik_%s_ctrl' % (self.name, num))
            b = coreUtils.addParent(c, 'group', '%s_ik_%s_buffer_srt' % (self.name, num))
            b.setParent(self.interfaceGrp)
            b.t.set(points[i])
            ikCtrls.append(c)
        ikCtrls[1].getParent().setParent(ikCtrls[0])
        ikCtrls[-2].getParent().setParent(ikCtrls[-1])

        # tangent vectors
        tangentVecs = []
        for i in range(len(ikCtrls)-1):
            num = str(i+1).zfill(2)
            vec = coreUtils.vectorBetweenNodes(ikCtrls[i], ikCtrls[i+1], name='%s_tangentVec_%s_utl' % (self.name, num))
            vecNorm = coreUtils.normalizeVector(vec.output3D, name='%s_tangentVecNorm_%s_utl' % (self.name, num))
            tangentVecs.append(vecNorm)

        tangentVecs.append(coreUtils.matrixAxisToVector(ikCtrls[-1], name='%s_tangentVec_%s_utl' % (self.name, str(len(points)).zfill(2))))

        # blended tangent vectors
        blendedtangentVecs = []
        jointMtxList = []

        for i in range(1, len(ikCtrls)-1):
            num = str(i+1).zfill(2)
            d = coreUtils.isDecomposed(ikCtrls[i])
            bc = coreUtils.blend(tangentVecs[i-1].output, tangentVecs[i].output, name='%s_blendedTangentVec_%s_utl' % (self.name, num))
            bc.blender.set(0.5)
            blendedtangentVecs.append(bc)

            # Joint Matrix
            mtx = pmc.createNode('fourByFourMatrix', name='%s_jointMtx_%s_utl' % (self.name, num))
            refVec = coreUtils.matrixAxisToVector(ikCtrls[i], name='%s_referenceVec_%s_utl' % (self.name, num))
            upVec = coreUtils.cross(bc.output, refVec.output, name='%s_upVec_%s_utl' % (self.name, num))
            sideVec = coreUtils.cross(upVec.output, bc.output, name='%s_sideVec_%s_utl' % (self.name, num))

            # Joint bend - radius/(dot(blendedTangent, tangent))
            dot = coreUtils.dot(tangentVecs[i].output, bc.output, name='%s_jointBend_%s_utl' % (self.name, num))
            md = coreUtils.divide(radius, dot.outputX, name='%s_jointRadius_%s_utl' % (self.name, num))
            localJointVec = coreUtils.minus([bc.output, d.outputTranslate], name='%s_blendedTangentVecToLocalSrt_%s_utl' % (self.name, num))
            vp = pmc.createNode('vectorProduct', name='%s_blendedTangentLocalNorm_%s_utl' % (self.name, num))
            vp.operation.set(0)
            vp.normalizeOutput.set(1)
            localJointVec.output3Dx.connect(vp.input1X)
            localJointVec.output3Dy.connect(vp.input1Y)
            vp.input1Z.set(0.001)
            blendAbs = coreUtils.forceAbsolute(vp.outputX, name='%s_absoluteBlend_%s' % (self.name, num))
            vp.outputY.connect(blendAbs[0].input1Y)
            blendAbs[0].outputY.connect(blendAbs[1].input1Y)
            blendAbs[0].input2Y.set(2)
            blendAbs[1].input2Y.set(0.5)
            sideBendBlend = coreUtils.blend(md.outputX, md.input1X, name='%s_sideBlend_%s_utl' % (self.name, num), blendAttr=blendAbs[1].outputX)
            sideVecScaled = pmc.createNode('multiplyDivide', name='%s_sideVecScaled_%s_utl' % (self.name, num))
            sideVec.output.connect(sideVecScaled.input1)
            coreUtils.connectAttrToMany(sideBendBlend.outputR, [sideVecScaled.input2X, sideVecScaled.input2Y, sideVecScaled.input2Z])
            upBendBlend = coreUtils.blend(md.outputX, md.input1X, name='%s_upBlend_%s_utl' % (self.name, num), blendAttr=blendAbs[1].outputY)
            upVecScaled = pmc.createNode('multiplyDivide', name='%s_upVecScaled_%s_utl' % (self.name, num))
            upVec.output.connect(upVecScaled.input1)
            coreUtils.connectAttrToMany(upBendBlend.outputR, [upVecScaled.input2X, upVecScaled.input2Y, upVecScaled.input2Z])

            sideVecScaled.outputX.connect(mtx.in00)
            sideVecScaled.outputY.connect(mtx.in01)
            sideVecScaled.outputZ.connect(mtx.in02)

            upVecScaled.outputX.connect(mtx.in10)
            upVecScaled.outputY.connect(mtx.in11)
            upVecScaled.outputZ.connect(mtx.in12)

            bc.outputR.connect(mtx.in20)
            bc.outputG.connect(mtx.in21)
            bc.outputB.connect(mtx.in22)

            d.outputTranslateX.connect(mtx.in30)
            d.outputTranslateY.connect(mtx.in31)
            d.outputTranslateZ.connect(mtx.in32)

            jointMtxList.append(mtx)

        posYPointsSharp = []
        negYPointsSharp = []
        posXPointsSharp = []
        negXPointsSharp = []

        posYPointsSharp.append(coreUtils.pointMatrixMult((0, 1, 0), ikCtrls[0].worldMatrix[0], name='%s_posY_sharpPoint_%s_utl' % (self.name, num)))
        negYPointsSharp.append(coreUtils.pointMatrixMult((0, -1, 0), ikCtrls[0].worldMatrix[0], name='%s_negY_sharpPoint_%s_utl' % (self.name, num)))
        posXPointsSharp.append(coreUtils.pointMatrixMult((1, 0, 0), ikCtrls[0].worldMatrix[0], name='%s_posX_sharpPoint_%s_utl' % (self.name, num)))
        negXPointsSharp.append(coreUtils.pointMatrixMult((-1, 0, 0), ikCtrls[0].worldMatrix[0], name='%s_negX_sharpPoint_%s_utl' % (self.name, num)))

        for i in range(len(jointMtxList)-1):
            divs = latticeDivisions*2
            exceptionList = [0, 1, len(jointMtxList)-2, len(jointMtxList)-3]
            if i in exceptionList:
                divs = latticeDivisions
            for a in range(divs):
                num = str((i*divs)+a).zfill(2)
                blendVal = (1.0 / (divs))*a

                mtxBlend = coreUtils.blendMatrices(jointMtxList[i].output, jointMtxList[i+1].output, 1-blendVal, blendVal, name='%s_mtxBlend_%s_utl' % (self.name, num))

                posYPointsSharp.append(coreUtils.pointMatrixMult((0, 1, 0), mtxBlend.matrixSum, name='%s_posY_sharpPoint_%s_utl' % (self.name, num)))
                negYPointsSharp.append(coreUtils.pointMatrixMult((0, -1, 0), mtxBlend.matrixSum, name='%s_negY_sharpPoint_%s_utl' % (self.name, num)))
                posXPointsSharp.append(coreUtils.pointMatrixMult((1, 0, 0), mtxBlend.matrixSum, name='%s_posX_sharpPoint_%s_utl' % (self.name, num)))
                negXPointsSharp.append(coreUtils.pointMatrixMult((-1, 0, 0), mtxBlend.matrixSum, name='%s_negX_sharpPoint_%s_utl' % (self.name, num)))

        posYPointsSharp.append(coreUtils.pointMatrixMult((0, 1, 0), ikCtrls[-1].worldMatrix[0], name='%s_posY_sharpPoint_%s_utl' % (self.name, num)))
        negYPointsSharp.append(coreUtils.pointMatrixMult((0, -1, 0), ikCtrls[-1].worldMatrix[0], name='%s_negY_sharpPoint_%s_utl' % (self.name, num)))
        posXPointsSharp.append(coreUtils.pointMatrixMult((1, 0, 0), ikCtrls[-1].worldMatrix[0], name='%s_posX_sharpPoint_%s_utl' % (self.name, num)))
        negXPointsSharp.append(coreUtils.pointMatrixMult((-1, 0, 0), ikCtrls[-1].worldMatrix[0], name='%s_negX_sharpPoint_%s_utl' % (self.name, num)))

        # create lattice
        latticeScale = math.sqrt(1*2)
        pmc.select([])
        lattice = pmc.lattice(dv=(2, 2, len(posYPointsSharp)), scale=(1, 1, 1))
        lattice[0].rename('%s_lattice_def' % self.name)
        lattice[1].rename('%s_lattice_cage' % self.name)
        lattice[2].rename('%s_lattice_base' % self.name)
        lattice[2].rz.set(45)
        lattice[2].s.set((latticeScale, latticeScale, coreUtils.getDistance(start, end)))

        for i in range(len(posYPointsSharp)):
            posYPointsSharp[i].output.connect(lattice[1].controlPoints[(i*4)+3])
            negYPointsSharp[i].output.connect(lattice[1].controlPoints[i*4])
            posXPointsSharp[i].output.connect(lattice[1].controlPoints[(i*4)+1])
            negXPointsSharp[i].output.connect(lattice[1].controlPoints[(i*4)+2])










    def cleanUp(self):
        pass
        #super(DrSnake, self).cleanUp()

