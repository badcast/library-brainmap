/*
 * author: badcast <lmecomposer@gmail.com>
 * year 2023
 * status: already coding
*/

#ifndef HEADER_SOURCE_H
#define HEADER_SOURCE_H

#include <iostream>

#define private public
#define protected public
#include <brain_map.hpp>
#undef private
#undef protected


using namespace std;
using namespace brain;

brain_map map = brain_map(100, 100);

std::size_t maxIterate = map.size();

brain_map::neuron_type *n1, *n2;
brain_map::site_type t1, t2;
std::size_t iterate = 0;
std::size_t errors = 0;

#endif
