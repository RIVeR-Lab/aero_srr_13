/**
 * @file   OccupancyGrid.cpp
 *
 * @date   May 7, 2013
 * @author parallels
 * @brief  \TODO
 */

/*
 * Copyright (c) 2013, RIVeR Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the <organization> nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//*********** SYSTEM DEPENDANCIES ****************//
#include <occupancy_grid/OccupancyGrid.hpp>
//************ LOCAL DEPENDANCIES ****************//

//***********    NAMESPACES     ****************//

using namespace occupancy_grid;
using namespace occupancy_grid::utilities;

//***************************************** CELLTRAIT *********************************************************//

CellTrait::CellTrait():
			enum_(UNKOWN)
{

}

CellTrait::CellTrait(Enum value):
				enum_(value)
{

}

CellTrait::CellTrait(int value)
{
	this->enum_ = enumFromValue(value);
}


CellTrait::Enum CellTrait::getEnum(void) const
{
	return this->enum_;
}

std::string CellTrait::getString(void) const
{
	return stringFromEnum(this->enum_);
}

int CellTrait::getValue(void) const
{
	return this->enum_;
}


std::ostream& CellTrait::operator<<(std::ostream& out) const
{
	out<<stringFromEnum(this->enum_);
	return out;
}

CellTrait& CellTrait::operator=(const CellTrait& rhs)
{
	this->enum_ = rhs.enum_;
	return *this;
}
CellTrait& CellTrait::operator=(const int& rhs)
{
	this->enum_ = enumFromValue(rhs);
	return *this;
}

bool CellTrait::operator==(const CellTrait& rhs) const
{
	return rhs.enum_ == this->enum_;
}
bool CellTrait::operator==(const int& rhs) const
{
	return this->enum_ == enumFromValue(rhs);
}

CellTrait::Enum CellTrait::enumFromValue(int value)
{
	switch(value)
	{
	case FREE_LOW_COST:
		return FREE_LOW_COST;
		break;
	case FREE_HIGH_COST:
		return FREE_HIGH_COST;
		break;
	case OBSTACLE:
		return OBSTACLE;
		break;
	case GOAL:
		return GOAL;
		break;
	case TRAVERSED:
		return TRAVERSED;
		break;
	default:
		return  UNKOWN;
		break;
	}
	return UNKOWN;
}

std::string CellTrait::stringFromEnum(Enum value)
{
	switch(value)
	{
	case FREE_LOW_COST:
		return "FREE_LOW_COST";
		break;
	case FREE_HIGH_COST:
		return "FREE_HIGH_COST";
		break;
	case OBSTACLE:
		return "OBSTACLE";
		break;
	case GOAL:
		return "GOAL";
		break;
	case TRAVERSED:
		return "TRAVERSED";
		break;
	default:
		return  "UNKOWN";
		break;
	}
	return "UNKOWN";
}
