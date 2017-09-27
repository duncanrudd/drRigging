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

        # create sharp matrices at each lattice division
        for i in range(len(ikCtrls)-1):
            num = str(i+1).zfill(2)
            divs = latticeDivisions*2
            exceptionList = [0, 1, len(ikCtrls)-2, len(ikCtrls)-3]
            if i in exceptionList:
                divs = latticeDivisions
            segMtx = pmc.createNode('fourByFourMatrix', name='%s_segMtx_%s_utl' % (self.name, num))

            refVec = coreUtils.matrixAxisToVector(ikCtrls[i], name='%s_referenceVec_%s_utl' % (self.name, num), axis='x')
            upVec = coreUtils.cross(tangentVecs[i].output, refVec.output, name='%s_upVec_%s_utl' % (self.name, num))
            sideVec = coreUtils.cross(upVec.output, tangentVecs[i].output, name='%s_sideVec_%s_utl' % (self.name, num))

            sideVec.outputX.connect(segMtx.in00)
            sideVec.outputY.connect(segMtx.in01)
            sideVec.outputZ.connect(segMtx.in02)

            upVec.outputX.connect(segMtx.in10)
            upVec.outputY.connect(segMtx.in11)
            upVec.outputZ.connect(segMtx.in12)

            tangentVecs[i].outputX.connect(segMtx.in20)
            tangentVecs[i].outputY.connect(segMtx.in21)
            tangentVecs[i].outputZ.connect(segMtx.in22)

            d = coreUtils.decomposeMatrix(ikCtrls[i].worldMatrix[0], name='%s_segMtxToSrt_%s_utl' % (self.name, num))
            d.outputTranslateX.connect(segMtx.in30)
            d.outputTranslateY.connect(segMtx.in31)
            d.outputTranslateZ.connect(segMtx.in32)

            dist = coreUtils.distanceBetweenNodes(ikCtrls[i], ikCtrls[i+1], name='%s_segLength_%s_utl' % (self.name, num))

            for a in range(divs):
                num = str((i*divs)+a+1).zfill(2)
                blendVal = (1.0 / (divs))*a

                grp = pmc.group(empty=1, name='%s_pointBase_%s_srt' % (self.name, num))
                grp.setParent(self.rigGrp)

                # Create matrix for base group
                '''
                yVec = coreUtils.matrixAxisToVector(localMtx.matrixSum, name='%s_pointYVec_%s_utl' % (self.name, num), axis='y')
                scaleY = coreUtils.divide(1.0, yVec.outputY, name='%s_pointScaleY_%s_utl' % (self.name, num))

                xVec = coreUtils.matrixAxisToVector(localMtx.matrixSum, name='%s_pointXVec_%s_utl' % (self.name, num), axis='x')
                scaleX = coreUtils.divide(1.0, xVec.outputX, name='%s_pointScaleX_%s_utl' % (self.name, num))
                '''

                mtx = None
                d = None
                if a == 0:
                    mtx = segMtx
                    d = coreUtils.decomposeMatrix(mtx.output, name='%s_pointMtxToSrt_%s_utl' % (self.name, num))
                else:
                    pointLocalOffset = coreUtils.convert(dist.distance, blendVal, name='%s_pointSegOffset_%s_utl' % (self.name, num))
                    localMtx = pmc.createNode('fourByFourMatrix', name='%s_pointLocalMtx_%s_utl' % (self.name, num))
                    pointLocalOffset.output.connect(localMtx.in32)
                    mtx = coreUtils.multiplyMatrices([localMtx.output, segMtx.output], name='%s_pointMtx_%s_utl' % (self.name, num))
                    d = coreUtils.decomposeMatrix(mtx.matrixSum, '%s_pointMtxToSrt_%s_utl' % (self.name, num))

                coreUtils.connectDecomposedMatrix(d, grp)











        '''
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
        '''










    def cleanUp(self):
        pass
        #super(DrSnake, self).cleanUp()

