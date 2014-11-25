/*
 * EGTable.h
 *
 *  Created on: Nov 19, 2014
 *      Author: perry
 */

#ifndef EGTABLE_H_
#define EGTABLE_H_

#include <openrave_env_generator/EnvironmentGenerator.h>

namespace TOService {

class EGTable : public EnvironmentGenerator {
public:
	EGTable(bool defaultTable_ = 0, int objects_ = 0);
	virtual ~EGTable();

protected:
	bool defaultTable;
	int objects;

	void generateEnvironment();
};

} /* namespace TOService */

#endif /* EGTABLE_H_ */
