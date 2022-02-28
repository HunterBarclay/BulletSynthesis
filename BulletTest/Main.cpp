#include <iostream>

#include "PhysicsManager.h"

int main() {

	btVector3* v = (btVector3*)calloc(1, sizeof(btVector3));
	*v = btVector3(1.0f, 3.0f, 7.0f);

	float* v_float = (float*)v;
	for (int i = 0; i < sizeof(btVector3) / sizeof(float); i++) {
		std::cout << "[" << i << "] -> " << v_float[i] << "\n";
	}

	delete v;

	return 0;
}
