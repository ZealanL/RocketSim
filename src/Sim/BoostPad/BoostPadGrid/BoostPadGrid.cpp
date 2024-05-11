#include "BoostPadGrid.h"

RS_NS_START

void BoostPadGrid::CheckCollision(Car* car) {
	if (car->_internalState.isDemoed || car->_internalState.boost >= 100)
		return;

	Vec carPos = car->_rigidBody.getWorldTransform().m_origin * BT_TO_UU;

	if (carPos.z > EXTENT_Z)
		return;

	int indexX = carPos.x / CELL_SIZE_X + (CELLS_X / 2);
	int indexY = carPos.y / CELL_SIZE_Y + (CELLS_Y / 2);

	for (int i = RS_MAX(indexX - 1, 0); i < CELLS_X; i++) {
		for (int j = RS_MAX(indexY - 1, 0); j < CELLS_Y; j++) {
			BoostPad* pad = pads[i][j];
			if (pad) {
				pad->_CheckCollide(car);
			}
		}
	}
}

void BoostPadGrid::Add(BoostPad* pad) {
	int indexX = pad->pos.x / CELL_SIZE_X + (CELLS_X / 2);
	int indexY = pad->pos.y / CELL_SIZE_Y + (CELLS_Y / 2);

	BoostPad*& ptrInArray = pads[indexX][indexY];
	if (ptrInArray != NULL) {
		RS_ERR_CLOSE(
			"BoostPadGrid::Add(): Failed to add a boost pad where there already was one " <<
			"(old: " << ptrInArray->pos << ", new: " << pad->pos << ") -> " <<
			"[" << indexX << ", " << indexY << "]"
		);
	} else {
		ptrInArray = pad;
	}
}

RS_NS_END