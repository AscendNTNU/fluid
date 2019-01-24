//
// Created by Simen Gangstad on 24.01.19
//

#include <string>

#ifndef FLUID_IDENTIFIABLE_H
#define FLUID_IDENTIFIABLE_H

namespace fluid {

	/**
	 * @brief      An interface for identifiable objects.
	 */
	class Identifiable {

	public: 
		const std::string identifier; 

		Identifiable(std::string id) : identifier(id) {}

		virtual ~Identifiable() = default; 
	};
}

#endif