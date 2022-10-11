/* Copyright (c) 2011-2014, Cyrille Favreau
 * All rights reserved. Do not distribute without permission.
 * Responsible Author: Cyrille Favreau <cyrille_favreau@hotmail.com>
 *
 * This file is part of Sol-R <https://github.com/cyrillefavreau/Sol-R>
 *
 * This library is free software; you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License version 3.0 as published
 * by the Free Software Foundation.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

#include "../Logging.h"
#include "../consts.h"
#include "PDBReader.h"

//#define CONNECTIONS

namespace solr
{
struct Element
{
    std::string Symbol;
    double ARENeg;
    double RCov;
    double RBO;
    double RVdW;
    int MaxBnd;
    double Mass;
    double ElNeg;
    double Ionization;
    double ElAffinity;
    double red;
    double green;
    double blue;
    std::string Name;
};

// https://github.com/openbabel/openbabel/blob/master/data/element.txt
const int NB_ELEMENTS = 119;
Element elements[NB_ELEMENTS] = { // Symb	ARENeg	RCov	RBO	RVdW	MaxBnd
                                  // Mass	ElNeg.	Ionization	ElAffinity	Red
                                  // Green	Blue	Name
    {"Xx", 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.07, 0.5, 0.7, "Dummy"},
    {"H", 2.2, 0.31, 0.31, 1.1, 1, 1.00794, 2.2, 13.5984, 0.75420375, 0.75,
     0.75, 0.75, "Hydrogen"},
    {"He", 0, 0.28, 0.28, 1.4, 0, 4.002602, 0, 24.5874, 0, 0.85, 1, 1,
     "Helium"},
    {"Li", 0.97, 1.28, 1.28, 1.81, 1, 6.941, 0.98, 5.3917, 0.618049, 0.8, 0.5,
     1, "Lithium"},
    {"Be", 1.47, 0.96, 0.96, 1.53, 2, 9.012182, 1.57, 9.3227, 0, 0.76, 1, 0,
     "Beryllium"},
    {"B", 2.01, 0.84, 0.84, 1.92, 4, 10.811, 2.04, 8.298, 0.279723, 1, 0.71,
     0.71, "Boron"},
    {"C", 2.5, 0.76, 0.76, 1.7, 4, 12.0107, 2.55, 11.2603, 1.262118, 0.4, 0.4,
     0.4, "Carbon"},
    {"N", 3.07, 0.71, 0.71, 1.55, 4, 14.0067, 3.04, 14.5341, -0.07, 0.05, 0.05,
     1, "Nitrogen"},
    {"O", 3.5, 0.66, 0.66, 1.52, 2, 15.9994, 3.44, 13.6181, 1.461112, 1, 0.05,
     0.05, "Oxygen"},
    {"F", 4.1, 0.57, 0.57, 1.47, 1, 18.9984032, 3.98, 17.4228, 3.4011887, 0.5,
     0.7, 1, "Fluorine"},
    {"Ne", 0, 0.58, 0.58, 1.54, 0, 20.1797, 0, 21.5645, 0, 0.7, 0.89, 0.96,
     "Neon"},
    {"Na", 1.01, 1.66, 1.66, 2.27, 1, 22.98977, 0.93, 5.1391, 0.547926, 0.67,
     0.36, 0.95, "Sodium"},
    {"Mg", 1.23, 1.41, 1.41, 1.73, 2, 24.305, 1.31, 7.6462, 0, 0.54, 1, 0,
     "Magnesium"},
    {"Al", 1.47, 1.21, 1.21, 1.84, 6, 26.981538, 1.61, 5.9858, 0.43283, 0.75,
     0.65, 0.65, "Aluminium"},
    {"Si", 1.74, 1.11, 1.11, 2.1, 6, 28.0855, 1.9, 8.1517, 1.389521, 0.5, 0.6,
     0.6, "Silicon"},
    {"P", 2.06, 1.07, 1.07, 1.8, 6, 30.973761, 2.19, 10.4867, 0.7465, 1, 0.5, 0,
     "Phosphorus"},
    {"S", 2.44, 1.05, 1.05, 1.8, 6, 32.065, 2.58, 10.36, 2.0771029, 0.7, 0.7, 0,
     "Sulfur"},
    {"Cl", 2.83, 1.02, 1.02, 1.75, 1, 35.453, 3.16, 12.9676, 3.612724, 0.12,
     0.94, 0.12, "Chlorine"},
    {"Ar", 0, 1.06, 1.06, 1.88, 0, 39.948, 0, 15.7596, 0, 0.5, 0.82, 0.89,
     "Argon"},
    {"K", 0.91, 2.03, 2.03, 2.75, 1, 39.0983, 0.82, 4.3407, 0.501459, 0.56,
     0.25, 0.83, "Potassium"},
    {"Ca", 1.04, 1.76, 1.76, 2.31, 2, 40.078, 1, 6.1132, 0.02455, 0.24, 1, 0,
     "Calcium"},
    {"Sc", 1.2, 1.7, 1.7, 2.3, 6, 44.95591, 1.36, 6.5615, 0.188, 0.9, 0.9, 0.9,
     "Scandium"},
    {"Ti", 1.32, 1.6, 1.6, 2.15, 6, 47.867, 1.54, 6.8281, 0.084, 0.75, 0.76,
     0.78, "Titanium"},
    {"V", 1.45, 1.53, 1.53, 2.05, 6, 50.9415, 1.63, 6.7462, 0.525, 0.65, 0.65,
     0.67, "Vanadium"},
    {"Cr", 1.56, 1.39, 1.39, 2.05, 6, 51.9961, 1.66, 6.7665, 0.67584, 0.54, 0.6,
     0.78, "Chromium"},
    {"Mn", 1.6, 1.39, 1.39, 2.05, 8, 54.938049, 1.55, 7.434, 0, 0.61, 0.48,
     0.78, "Manganese"},
    {"Fe", 1.64, 1.32, 1.32, 2.05, 6, 55.845, 1.83, 7.9024, 0.151, 0.88, 0.4,
     0.2, "Iron"},
    {"Co", 1.7, 1.26, 1.26, 2, 6, 58.9332, 1.88, 7.881, 0.6633, 0.94, 0.56,
     0.63, "Cobalt"},
    {"Ni", 1.75, 1.24, 1.24, 2, 6, 58.6934, 1.91, 7.6398, 1.15716, 0.31, 0.82,
     0.31, "Nickel"},
    {"Cu", 1.75, 1.32, 1.32, 2, 6, 63.546, 1.9, 7.7264, 1.23578, 0.78, 0.5, 0.2,
     "Copper"},
    {"Zn", 1.66, 1.22, 1.22, 2.1, 6, 65.38, 1.65, 9.3942, 0, 0.49, 0.5, 0.69,
     "Zinc"},
    {"Ga", 1.82, 1.22, 1.22, 1.87, 3, 69.723, 1.81, 5.9993, 0.41, 0.76, 0.56,
     0.56, "Gallium"},
    {"Ge", 2.02, 1.2, 1.2, 2.11, 4, 72.64, 2.01, 7.8994, 1.232712, 0.4, 0.56,
     0.56, "Germanium"},
    {"As", 2.2, 1.19, 1.19, 1.85, 3, 74.9216, 2.18, 9.7886, 0.814, 0.74, 0.5,
     0.89, "Arsenic"},
    {"Se", 2.48, 1.2, 1.2, 1.9, 2, 78.96, 2.55, 9.7524, 2.02067, 1, 0.63, 0,
     "Selenium"},
    {"Br", 2.74, 1.2, 1.2, 1.83, 1, 79.904, 2.96, 11.8138, 3.363588, 0.65, 0.16,
     0.16, "Bromine"},
    {"Kr", 0, 1.16, 1.16, 2.02, 0, 83.798, 3, 13.9996, 0, 0.36, 0.72, 0.82,
     "Krypton"},
    {"Rb", 0.89, 2.2, 2.2, 3.03, 1, 85.4678, 0.82, 4.1771, 0.485916, 0.44, 0.18,
     0.69, "Rubidium"},
    {"Sr", 0.99, 1.95, 1.95, 2.49, 2, 87.62, 0.95, 5.6949, 0.05206, 0, 1, 0,
     "Strontium"},
    {"Y", 1.11, 1.9, 1.9, 2.4, 6, 88.90585, 1.22, 6.2173, 0.307, 0.58, 1, 1,
     "Yttrium"},
    {"Zr", 1.22, 1.75, 1.75, 2.3, 6, 91.224, 1.33, 6.6339, 0.426, 0.58, 0.88,
     0.88, "Zirconium"},
    {"Nb", 1.23, 1.64, 1.64, 2.15, 6, 92.90638, 1.6, 6.7589, 0.893, 0.45, 0.76,
     0.79, "Niobium"},
    {"Mo", 1.3, 1.54, 1.54, 2.1, 6, 95.96, 2.16, 7.0924, 0.7472, 0.33, 0.71,
     0.71, "Molybdenum"},
    {"Tc", 1.36, 1.47, 1.47, 2.05, 6, 98, 1.9, 7.28, 0.55, 0.23, 0.62, 0.62,
     "Technetium"},
    {"Ru", 1.42, 1.46, 1.46, 2.05, 6, 101.07, 2.2, 7.3605, 1.04638, 0.14, 0.56,
     0.56, "Ruthenium"},
    {"Rh", 1.45, 1.42, 1.42, 2, 6, 102.9055, 2.28, 7.4589, 1.14289, 0.04, 0.49,
     0.55, "Rhodium"},
    {"Pd", 1.35, 1.39, 1.39, 2.05, 6, 106.42, 2.2, 8.3369, 0.56214, 0, 0.41,
     0.52, "Palladium"},
    {"Ag", 1.42, 1.45, 1.45, 2.1, 6, 107.8682, 1.93, 7.5762, 1.30447, 0.88,
     0.88, 1, "Silver"},
    {"Cd", 1.46, 1.44, 1.44, 2.2, 6, 112.411, 1.69, 8.9938, 0, 1, 0.85, 0.56,
     "Cadmium"},
    {"In", 1.49, 1.42, 1.42, 2.2, 3, 114.818, 1.78, 5.7864, 0.404, 0.65, 0.46,
     0.45, "Indium"},
    {"Sn", 1.72, 1.39, 1.39, 1.93, 4, 118.701, 1.96, 7.3439, 1.112066, 0.4, 0.5,
     0.5, "Tin"},
    {"Sb", 1.82, 1.39, 1.39, 2.17, 3, 121.76, 2.05, 8.6084, 1.047401, 0.62,
     0.39, 0.71, "Antimony"},
    {"Te", 2.01, 1.38, 1.38, 2.06, 2, 127.6, 2.1, 9.0096, 1.970875, 0.83, 0.48,
     0, "Tellurium"},
    {"I", 2.21, 1.39, 1.39, 1.98, 1, 126.90447, 2.66, 10.4513, 3.059038, 0.58,
     0, 0.58, "Iodine"},
    {"Xe", 0, 1.4, 1.4, 2.16, 0, 131.293, 2.6, 12.1298, 0, 0.26, 0.62, 0.69,
     "Xenon"},
    {"Cs", 0.86, 2.44, 2.44, 3.43, 1, 132.90545, 0.79, 3.8939, 0.471626, 0.34,
     0.09, 0.56, "Caesium"},
    {"Ba", 0.97, 2.15, 2.15, 2.68, 2, 137.327, 0.89, 5.2117, 0.14462, 0, 0.79,
     0, "Barium"},
    {"La", 1.08, 2.07, 2.07, 2.5, 12, 138.9055, 1.1, 5.5769, 0.47, 0.44, 0.83,
     1, "Lanthanum"},
    {"Ce", 0, 2.04, 2.04, 2.48, 6, 140.116, 1.12, 5.5387, 0.5, 1, 1, 0.78,
     "Cerium"},
    {"Pr", 0, 2.03, 2.03, 2.47, 6, 140.90765, 1.13, 5.473, 0.5, 0.85, 1, 0.78,
     "Praseodymium"},
    {"Nd", 0, 2.01, 2.01, 2.45, 6, 144.24, 1.14, 5.525, 0.5, 0.78, 1, 0.78,
     "Neodymium"},
    {"Pm", 0, 1.99, 1.99, 2.43, 6, 145, 0, 5.582, 0.5, 0.64, 1, 0.78,
     "Promethium"},
    {"Sm", 0, 1.98, 1.98, 2.42, 6, 150.36, 1.17, 5.6437, 0.5, 0.56, 1, 0.78,
     "Samarium"},
    {"Eu", 0, 1.98, 1.98, 2.4, 6, 151.964, 0, 5.6704, 0.5, 0.38, 1, 0.78,
     "Europium"},
    {"Gd", 0, 1.96, 1.96, 2.38, 6, 157.25, 1.2, 6.1498, 0.5, 0.27, 1, 0.78,
     "Gadolinium"},
    {"Tb", 0, 1.94, 1.94, 2.37, 6, 158.92534, 0, 5.8638, 0.5, 0.19, 1, 0.78,
     "Terbium"},
    {"Dy", 0, 1.92, 1.92, 2.35, 6, 162.5, 1.22, 5.9389, 0.5, 0.12, 1, 0.78,
     "Dysprosium"},
    {"Ho", 0, 1.92, 1.92, 2.33, 6, 164.93032, 1.23, 6.0215, 0.5, 0, 1, 0.61,
     "Holmium"},
    {"Er", 0, 1.89, 1.89, 2.32, 6, 167.259, 1.24, 6.1077, 0.5, 0, 0.9, 0.46,
     "Erbium"},
    {"Tm", 0, 1.9, 1.9, 2.3, 6, 168.93421, 1.25, 6.1843, 0.5, 0, 0.83, 0.32,
     "Thulium"},
    {"Yb", 0, 1.87, 1.87, 2.28, 6, 173.054, 0, 6.2542, 0.5, 0, 0.75, 0.22,
     "Ytterbium"},
    {"Lu", 0, 1.87, 1.87, 2.27, 6, 174.9668, 1.27, 5.4259, 0.5, 0, 0.67, 0.14,
     "Lutetium"},
    {"Hf", 1.23, 1.75, 1.75, 2.25, 6, 178.49, 1.3, 6.8251, 0, 0.3, 0.76, 1,
     "Hafnium"},
    {"Ta", 1.33, 1.7, 1.7, 2.2, 6, 180.9479, 1.5, 7.5496, 0.322, 0.3, 0.65, 1,
     "Tantalum"},
    {"W", 1.4, 1.62, 1.62, 2.1, 6, 183.84, 2.36, 7.864, 0.815, 0.13, 0.58, 0.84,
     "Tungsten"},
    {"Re", 1.46, 1.51, 1.51, 2.05, 6, 186.207, 1.9, 7.8335, 0.15, 0.15, 0.49,
     0.67, "Rhenium"},
    {"Os", 1.52, 1.44, 1.44, 2, 6, 190.23, 2.2, 8.4382, 1.0778, 0.15, 0.4, 0.59,
     "Osmium"},
    {"Ir", 1.55, 1.41, 1.41, 2, 6, 192.217, 2.2, 8.967, 1.56436, 0.09, 0.33,
     0.53, "Iridium"},
    {"Pt", 1.44, 1.36, 1.36, 2.05, 6, 195.078, 2.28, 8.9588, 2.1251, 0.9, 0.85,
     0.68, "Platinum"},
    {"Au", 1.42, 1.36, 1.36, 2.1, 6, 196.96655, 2.54, 9.2255, 2.30861, 0.8,
     0.82, 0.12, "Gold"},
    {"Hg", 1.44, 1.32, 1.32, 2.05, 6, 200.59, 2, 10.4375, 0, 0.71, 0.71, 0.76,
     "Mercury"},
    {"Tl", 1.44, 1.45, 1.45, 1.96, 3, 204.3833, 1.62, 6.1082, 0.377, 0.65, 0.33,
     0.3, "Thallium"},
    {"Pb", 1.55, 1.46, 1.46, 2.02, 4, 207.2, 2.33, 7.4167, 0.364, 0.34, 0.35,
     0.38, "Lead"},
    {"Bi", 1.67, 1.48, 1.48, 2.07, 3, 208.9804, 2.02, 7.2855, 0.942363, 0.62,
     0.31, 0.71, "Bismuth"},
    {"Po", 1.76, 1.4, 1.4, 1.97, 2, 209, 2, 8.414, 1.9, 0.67, 0.36, 0,
     "Polonium"},
    {"At", 1.9, 1.5, 1.5, 2.02, 1, 210, 2.2, 0, 2.8, 0.46, 0.31, 0.27,
     "Astatine"},
    {"Rn", 0, 1.5, 1.5, 2.2, 0, 222, 0, 10.7485, 0, 0.26, 0.51, 0.59, "Radon"},
    {"Fr", 0, 2.6, 2.6, 3.48, 1, 223, 0.7, 4.0727, 0, 0.26, 0, 0.4, "Francium"},
    {"Ra", 0, 2.21, 2.21, 2.83, 2, 226, 0.9, 5.2784, 0, 0, 0.49, 0, "Radium"},
    {"Ac", 0, 2.15, 2.15, 2, 6, 227, 1.1, 5.17, 0, 0.44, 0.67, 0.98,
     "Actinium"},
    {"Th", 0, 2.06, 2.06, 2.4, 6, 232.0381, 1.3, 6.3067, 0, 0, 0.73, 1,
     "Thorium"},
    {"Pa", 0, 2, 2, 2, 6, 231.03588, 1.5, 5.89, 0, 0, 0.63, 1, "Protactinium"},
    {"U", 0, 1.96, 1.96, 2.3, 6, 238.02891, 1.38, 6.1941, 0, 0, 0.56, 1,
     "Uranium"},
    {"Np", 0, 1.9, 1.9, 2, 6, 237.05, 1.36, 6.2657, 0, 0, 0.5, 1, "Neptunium"},
    {"Pu", 0, 1.87, 1.87, 2, 6, 244.06, 1.28, 6.026, 0, 0, 0.42, 1,
     "Plutonium"},
    {"Am", 0, 1.8, 1.8, 2, 6, 243.06, 1.3, 5.9738, 0, 0.33, 0.36, 0.95,
     "Americium"},
    {"Cm", 0, 1.69, 1.69, 2, 6, 247.07, 1.3, 5.9914, 0, 0.47, 0.36, 0.89,
     "Curium"},
    {"Bk", 0, 1.6, 1.6, 2, 6, 247.07, 1.3, 6.1979, 0, 0.54, 0.31, 0.89,
     "Berkelium"},
    {"Cf", 0, 1.6, 1.6, 2, 6, 251.08, 1.3, 6.2817, 0, 0.63, 0.21, 0.83,
     "Californium"},
    {"Es", 0, 1.6, 1.6, 2, 6, 252.08, 1.3, 6.42, 0, 0.7, 0.12, 0.83,
     "Einsteinium"},
    {"Fm", 0, 1.6, 1.6, 2, 6, 257.1, 1.3, 6.5, 0, 0.7, 0.12, 0.73, "Fermium"},
    {"Md", 0, 1.6, 1.6, 2, 6, 258.1, 1.3, 6.58, 0, 0.7, 0.05, 0.65,
     "Mendelevium"},
    {"No", 0, 1.6, 1.6, 2, 6, 259.1, 1.3, 6.65, 0, 0.74, 0.05, 0.53,
     "Nobelium"},
    {"Lr", 0, 1.6, 1.6, 2, 6, 262.11, 0, 4.9, 0, 0.78, 0, 0.4, "Lawrencium"},
    {"Rf", 0, 1.6, 1.6, 2, 6, 265.12, 0, 6, 0, 0.8, 0, 0.35, "Rutherfordium"},
    {"Db", 0, 1.6, 1.6, 2, 6, 268.13, 0, 0, 0, 0.82, 0, 0.31, "Dubnium"},
    {"Sg", 0, 1.6, 1.6, 2, 6, 271.13, 0, 0, 0, 0.85, 0, 0.27, "Seaborgium"},
    {"Bh", 0, 1.6, 1.6, 2, 6, 270, 0, 0, 0, 0.88, 0, 0.22, "Bohrium"},
    {"Hs", 0, 1.6, 1.6, 2, 6, 277.15, 0, 0, 0, 0.9, 0, 0.18, "Hassium"},
    {"Mt", 0, 1.6, 1.6, 2, 6, 276.15, 0, 0, 0, 0.92, 0, 0.15, "Meitnerium"},
    {"Ds", 0, 1.6, 1.6, 2, 6, 281.16, 0, 0, 0, 0.93, 0, 0.14, "Darmstadtium"},
    {"Rg", 0, 1.6, 1.6, 2, 6, 280.16, 0, 0, 0, 0.94, 0, 0.13, "Roentgenium"},
    {"Cn", 0, 1.6, 1.6, 2, 6, 285.17, 0, 0, 0, 0.95, 0, 0.12, "Copernicium"},
    {"Uut", 0, 1.6, 1.6, 2, 6, 284.18, 0, 0, 0, 0.96, 0, 0.11, "Ununtrium"},
    {"Fl", 0, 1.6, 1.6, 2, 6, 289.19, 0, 0, 0, 0.97, 0, 0.1, "Flerovium"},
    {"Uup", 0, 1.6, 1.6, 2, 6, 288.19, 0, 0, 0, 0.98, 0, 0.09, "Ununpentium"},
    {"Lv", 0, 1.6, 1.6, 2, 6, 293, 0, 0, 0, 0.99, 0, 0.08, "Livermorium"},
    {"Uuh", 0, 1.6, 1.6, 2, 6, 294, 0, 0, 0, 0.99, 0, 0.07, "Ununseptium"},
    {"Uuh", 0, 1.6, 1.6, 2, 6, 294, 0, 0, 0, 0.99, 0, 0.06, "Ununoctium"}};

struct ColorMap
{
    std::string symbol;
    short R, G, B;
};
ColorMap colormap[NB_ELEMENTS] = {
    {"H", 0xFF, 0xFF, 0xFF},  {"He", 0xD9, 0xFF, 0xFF},
    {"Li", 0xCC, 0x80, 0xFF}, {"Be", 0xC2, 0xFF, 0x00},
    {"B", 0xFF, 0xB5, 0xB5},  {"C", 0x90, 0x90, 0x90},
    {"N", 0x30, 0x50, 0xF8},  {"O", 0xFF, 0x0D, 0x0D},
    {"F", 0x9E, 0x05, 0x1},   {"Ne", 0xB3, 0xE3, 0xF5},
    {"Na", 0xAB, 0x5C, 0xF2}, {"Mg", 0x8A, 0xFF, 0x00},
    {"Al", 0xBF, 0xA6, 0xA6}, {"Si", 0xF0, 0xC8, 0xA0},
    {"P", 0xFF, 0x80, 0x00},  {"S", 0xFF, 0xFF, 0x30},
    {"Cl", 0x1F, 0xF0, 0x1F}, {"Ar", 0x80, 0xD1, 0xE3},
    {"K", 0x8F, 0x40, 0xD4},  {"Ca", 0x3D, 0xFF, 0x00},
    {"Sc", 0xE6, 0xE6, 0xE6}, {"Ti", 0xBF, 0xC2, 0xC7},
    {"V", 0xA6, 0xA6, 0xAB},  {"Cr", 0x8A, 0x99, 0xC7},
    {"Mn", 0x9C, 0x7A, 0xC7}, {"Fe", 0xE0, 0x66, 0x33},
    {"Co", 0xF0, 0x90, 0xA0}, {"Ni", 0x50, 0xD0, 0x50},
    {"Cu", 0xC8, 0x80, 0x33}, {"Zn", 0x7D, 0x80, 0xB0},
    {"Ga", 0xC2, 0x8F, 0x8F}, {"Ge", 0x66, 0x8F, 0x8F},
    {"As", 0xBD, 0x80, 0xE3}, {"Se", 0xFF, 0xA1, 0x00},
    {"Br", 0xA6, 0x29, 0x29}, {"Kr", 0x5C, 0xB8, 0xD1},
    {"Rb", 0x70, 0x2E, 0xB0}, {"Sr", 0x00, 0xFF, 0x00},
    {"Y", 0x94, 0xFF, 0xFF},  {"Zr", 0x94, 0xE0, 0xE0},
    {"Nb", 0x73, 0xC2, 0xC9}, {"Mo", 0x54, 0xB5, 0xB5},
    {"Tc", 0x3B, 0x9E, 0x9E}, {"Ru", 0x24, 0x8F, 0x8F},
    {"Rh", 0x0A, 0x7D, 0x8C}, {"Pd", 0x69, 0x85, 0x00},
    {"Ag", 0xC0, 0xC0, 0xC0}, {"Cd", 0xFF, 0xD9, 0x8F},
    {"In", 0xA6, 0x75, 0x73}, {"Sn", 0x66, 0x80, 0x80},
    {"Sb", 0x9E, 0x63, 0xB5}, {"Te", 0xD4, 0x7A, 0x00},
    {"I", 0x94, 0x00, 0x94},  {"Xe", 0x42, 0x9E, 0xB0},
    {"Cs", 0x57, 0x17, 0x8F}, {"Ba", 0x00, 0xC9, 0x00},
    {"La", 0x70, 0xD4, 0xFF}, {"Ce", 0xFF, 0xFF, 0xC7},
    {"Pr", 0xD9, 0xFF, 0xC7}, {"Nd", 0xC7, 0xFF, 0xC7},
    {"Pm", 0xA3, 0xFF, 0xC7}, {"Sm", 0x8F, 0xFF, 0xC7},
    {"Eu", 0x61, 0xFF, 0xC7}, {"Gd", 0x45, 0xFF, 0xC7},
    {"Tb", 0x30, 0xFF, 0xC7}, {"Dy", 0x1F, 0xFF, 0xC7},
    {"Ho", 0x00, 0xFF, 0x9C}, {"Er", 0x00, 0xE6, 0x75},
    {"Tm", 0x00, 0xD4, 0x52}, {"Yb", 0x00, 0xBF, 0x38},
    {"Lu", 0x00, 0xAB, 0x24}, {"Hf", 0x4D, 0xC2, 0xFF},
    {"Ta", 0x4D, 0xA6, 0xFF}, {"W", 0x21, 0x94, 0xD6},
    {"Re", 0x26, 0x7D, 0xAB}, {"Os", 0x26, 0x66, 0x96},
    {"Ir", 0x17, 0x54, 0x87}, {"Pt", 0xD0, 0xD0, 0xE0},
    {"Au", 0xFF, 0xD1, 0x23}, {"Hg", 0xB8, 0xB8, 0xD0},
    {"Tl", 0xA6, 0x54, 0x4D}, {"Pb", 0x57, 0x59, 0x61},
    {"Bi", 0x9E, 0x4F, 0xB5}, {"Po", 0xAB, 0x5C, 0x00},
    {"At", 0x75, 0x4F, 0x45}, {"Rn", 0x42, 0x82, 0x96},
    {"Fr", 0x42, 0x00, 0x66}, {"Ra", 0x00, 0x7D, 0x00},
    {"Ac", 0x70, 0xAB, 0xFA}, {"Th", 0x00, 0xBA, 0xFF},
    {"Pa", 0x00, 0xA1, 0xFF}, {"U", 0x00, 0x8F, 0xFF},
    {"Np", 0x00, 0x80, 0xFF}, {"Pu", 0x00, 0x6B, 0xFF},
    {"Am", 0x54, 0x5C, 0xF2}, {"Cm", 0x78, 0x5C, 0xE3},
    {"Bk", 0x8A, 0x4F, 0xE3}, {"Cf", 0xA1, 0x36, 0xD4},
    {"Es", 0xB3, 0x1F, 0xD4}, {"Fm", 0xB3, 0x1F, 0xBA},
    {"Md", 0xB3, 0x0D, 0xA6}, {"No", 0xBD, 0x0D, 0x87},
    {"Lr", 0xC7, 0x00, 0x66}, {"Rf", 0xCC, 0x00, 0x59},
    {"Db", 0xD1, 0x00, 0x4F}, {"Sg", 0xD9, 0x00, 0x45},
    {"Bh", 0xE0, 0x00, 0x38}, {"Hs", 0xE6, 0x00, 0x2E},
    {"Mt", 0xEB, 0x00, 0x26}};

/*
struct Element
{
std::string name;
float radius;
int materialId;
};
*/

struct Atom
{
    int processed;
    int id;
    int index;
    vec4f position;
    int materialId;
    int chainId;
    int residue;
    bool isBackbone;
    bool isWater;
};

struct Connection
{
    int atom1;
    int atom2;
};

const float DEFAULT_ATOM_DISTANCE = 30.f;
const float DEFAULT_STICK_DISTANCE = 1.7f;

struct AtomicRadius
{
    std::string Symbol;
    float radius;
    int index;
};

const AtomicRadius atomic_radius[NB_ELEMENTS] = {{"C", 67.f, 1},
                                                 {"N", 56.f, 2},
                                                 {"O", 48.f, 3},
                                                 {"H", 53.f, 4},
                                                 {"B", 87.f, 5},
                                                 {"F", 42.f, 6},
                                                 {"P", 98.f, 7},
                                                 {"S", 88.f, 8},
                                                 {"V", 171.f, 9},
                                                 {"K", 243.f, 10},
                                                 {"HE", 31.f, 11},
                                                 {"LI", 167.f, 12},
                                                 {"BE", 112.f, 13},
                                                 {"NE", 38.f, 14},
                                                 {"NA", 190.f, 15},
                                                 {"MG", 145.f, 16},
                                                 {"AL", 118.f, 17},
                                                 {"SI", 111.f, 18},
                                                 {"CL", 79.f, 19},
                                                 {"AR", 71.f, 20},
                                                 {"CA", 194.f, 21},
                                                 {"SC", 184.f, 22},
                                                 {"TI", 176.f, 23},
                                                 {"CR", 166.f, 24},
                                                 {"MN", 161.f, 25},
                                                 {"FE", 156.f, 26},
                                                 {"CO", 152.f, 27},
                                                 {"NI", 149.f, 28},
                                                 {"CU", 145.f, 29},
                                                 {"ZN", 142.f, 30},
                                                 {"GA", 136.f, 31},
                                                 {"GE", 125.f, 32},
                                                 {"AS", 114.f, 33},
                                                 {"SE", 103.f, 34},
                                                 {"BR", 94.f, 35},
                                                 {"KR", 88.f, 36},

                                                 // TODO
                                                 {"OD1", 25.f, 37},
                                                 {"OD2", 25.f, 38},
                                                 {"CG1", 25.f, 39},
                                                 {"CG2", 25.f, 40},
                                                 {"CD1", 25.f, 41},
                                                 {"CB", 25.f, 42},
                                                 {"CG", 25.f, 43},
                                                 {"CD", 25.f, 44},
                                                 {"OE1", 25.f, 45},
                                                 {"NE2", 25.f, 46},
                                                 {"CZ", 25.f, 47},
                                                 {"NH1", 25.f, 48},
                                                 {"NH2", 25.f, 49},
                                                 {"CD2", 25.f, 50},
                                                 {"CE1", 25.f, 51},
                                                 {"CE2", 25.f, 52},
                                                 {"CE", 25.f, 53},
                                                 {"NZ", 25.f, 54},
                                                 {"OH", 25.f, 55},
                                                 {"CE", 25.f, 56},
                                                 {"ND1", 25.f, 57},
                                                 {"ND2", 25.f, 58},
                                                 {"OXT", 25.f, 59},
                                                 {"OG1", 25.f, 60},
                                                 {"NE1", 25.f, 61},
                                                 {"CE3", 25.f, 62},
                                                 {"CZ2", 25.f, 63},
                                                 {"CZ3", 25.f, 64},
                                                 {"CH2", 25.f, 65},
                                                 {"OE2", 25.f, 66},
                                                 {"OG", 25.f, 67},
                                                 {"OE2", 25.f, 68},
                                                 {"SD", 25.f, 69},
                                                 {"SG", 25.f, 70},
                                                 {"C1*", 25.f, 71},
                                                 {"C2", 25.f, 72},
                                                 {"C2*", 25.f, 73},
                                                 {"C3*", 25.f, 74},
                                                 {"C4", 25.f, 75},
                                                 {"C4*", 25.f, 76},
                                                 {"C5", 25.f, 77},
                                                 {"C5*", 25.f, 78},
                                                 {"C5M", 25.f, 79},
                                                 {"C6", 25.f, 80},
                                                 {"C8", 25.f, 81},
                                                 {"H1", 25.f, 82},
                                                 {"H1*", 25.f, 83},
                                                 {"H2", 25.f, 84},
                                                 {"H2*", 25.f, 85},
                                                 {"H3", 25.f, 86},
                                                 {"H3*", 25.f, 87},
                                                 {"H3P", 25.f, 88},
                                                 {"H4", 25.f, 89},
                                                 {"H4*", 25.f, 90},
                                                 {"H5", 25.f, 91},
                                                 {"H5*", 25.f, 92},
                                                 {"H5M", 25.f, 93},
                                                 {"H6", 25.f, 94},
                                                 {"H8", 25.f, 95},
                                                 {"N1", 25.f, 96},
                                                 {"N2", 25.f, 97},
                                                 {"N3", 25.f, 98},
                                                 {"N4", 25.f, 99},
                                                 {"N6", 25.f, 100},
                                                 {"N7", 25.f, 101},
                                                 {"N9", 25.f, 102},
                                                 {"O1P", 25.f, 103},
                                                 {"O2", 25.f, 104},
                                                 {"O2P", 25.f, 105},
                                                 {"O3*", 25.f, 106},
                                                 {"O3P", 25.f, 107},
                                                 {"O4", 25.f, 108},
                                                 {"O4*", 25.f, 109},
                                                 {"O5*", 25.f, 110},
                                                 {"O6", 25.f, 111},
                                                 {"OXT", 25.f, 112},
                                                 {"P", 25.f, 113}};

PDBReader::PDBReader(void)
    : m_nbPrimitives(0)
    , m_nbBoxes(0)
{
}

PDBReader::~PDBReader(void) {}

vec4f PDBReader::loadAtomsFromFile(const std::string &filename,
                                   GPUKernel &cudaKernel,
                                   GeometryType geometryType,
                                   const float defaultAtomSize,
                                   const float defaultStickSize,
                                   const int materialType, const vec4f scale,
                                   const bool useModels)
{
    LOG_INFO(1, "OBJ Filename.......: " << filename);

    for (int i = 0; i < NB_ELEMENTS; ++i)
    {
        std::transform(colormap[i].symbol.begin(), colormap[i].symbol.end(),
                       colormap[i].symbol.begin(), ::toupper);
        cudaKernel.setMaterial(i, static_cast<float>(colormap[i].R) / 255.f,
                               static_cast<float>(colormap[i].G) / 255.f,
                               static_cast<float>(colormap[i].B) / 255.f, 0.f,
                               0.f, 0.f, false, false, 0, 0.f, 0.f,
                               TEXTURE_NONE, TEXTURE_NONE, TEXTURE_NONE,
                               TEXTURE_NONE, TEXTURE_NONE, TEXTURE_NONE,
                               TEXTURE_NONE, 1.f, 100.f, 0.f, 0.f,
                               cudaKernel.getSceneInfo().viewDistance, 0.f,
                               false);
    }

    cudaKernel.resetBoxes(true);

    float distanceRatio = 2.f;

    std::map<int, Atom> atoms;
    std::vector<Connection> connections;
    vec4f minPos = make_vec4f(100000.f, 100000.f, 100000.f);
    vec4f maxPos = make_vec4f(-100000.f, -100000.f, -100000.f, 0.f);
    LOG_INFO(1,
             "-----------------------------------------------------------------"
             "---------------");
    LOG_INFO(1, "Loading PDB File: " << filename);
    int index(0);
    std::ifstream file(filename.c_str());
    if (file.is_open())
    {
        while (file.good())
        {
            std::string line;
            std::string value;
            std::getline(file, line);
            if (line.find("ATOM") == 0 /* || line.find("HETATM") == 0 */)
            {
                // Atom
                Atom atom;
                atom.index = index;
                index++;
                std::string atomName;
                std::string chainId;
                std::string atomCode;
                size_t i(0);
                std::string A, B, C, D, E, F, G, H, I, J, K, L;
                while (i < line.length())
                {
                    switch (i)
                    {
                    case 6: // ID
                    case 12:
                    case 76: // Atom name
                    case 22: // ChainID
                    case 30: // x
                    case 38: // y
                    case 46: // z
                        value = "";
                        break;
                    case 21:
                        atom.chainId = (int)line.at(i) - 64;
                        break;
                    case 11:
                        atom.id = static_cast<int>(atoi(value.c_str()));
                        break;
                    case 17:
                        atomCode = value;
                        break;
                    case 79:
                        atomName = value;
                        break;
                    case 26:
                        atom.residue = static_cast<int>(atoi(value.c_str()));
                        break;
                    case 37:
                        atom.position.x =
                            static_cast<float>(atof(value.c_str()));
                        break;
                    case 45:
                        atom.position.y =
                            static_cast<float>(atof(value.c_str()));
                        break;
                    case 53:
                        atom.position.z =
                            -static_cast<float>(atof(value.c_str()));
                        break;
                    default:
                        if (line.at(i) != ' ')
                            value += line.at(i);
                        break;
                    }
                    i++;
                }

                LOG_INFO(3, "Atom: " << atomName)
                // Backbone
                atom.isBackbone =
                    (geometryType == gtBackbone ||
                     geometryType == gtIsoSurface || atomCode.length() == 1);

                // Material
                atom.materialId = 0;
                i = 0;
                bool found(false);
                while (!found && i < NB_ELEMENTS)
                {
                    if (atomName == colormap[i].symbol)
                    {
                        found = true;
                        switch (materialType)
                        {
                        case 1:
                            atom.materialId = (atom.chainId % 2 == 0)
                                                  ? static_cast<int>(i)
                                                  : 1000;
                            break;
                        case 2:
                            atom.materialId = atom.residue % 10;
                            break;
                        default:
                            atom.materialId = static_cast<int>(i);
                            break;
                        }
                        atom.position.w = (geometryType == gtFixedSizeAtoms)
                                              ? defaultAtomSize
                                              : 0.5f * defaultAtomSize;
                    }
                    ++i;
                }
                if (!found)
                {
                    LOG_ERROR("Could not find atomic color for '" << atomCode
                                                                  << "'");
                }

                // Radius
                if (geometryType == gtFixedSizeAtoms)
                {
                    atom.position.w = defaultAtomSize;
                }
                else
                {
                    i = 0;
                    found = false;
                    while (!found && i < NB_ELEMENTS)
                    {
                        if (atomName == atomic_radius[i].Symbol)
                        {
                            atom.position.w = atomic_radius[i].radius;
                            found = true;
                        }
                        ++i;
                    }
                    if (!found)
                        LOG_ERROR("Could not find atomic radius for '"
                                  << atomCode << "'");
                }

                if (geometryType != gtBackbone || atom.isBackbone)
                {
                    // Compute molecule size
                    // min
                    minPos.x = (atom.position.x < minPos.x) ? atom.position.x
                                                            : minPos.x;
                    minPos.y = (atom.position.y < minPos.y) ? atom.position.y
                                                            : minPos.y;
                    minPos.z = (atom.position.z < minPos.z) ? atom.position.z
                                                            : minPos.z;

                    // max
                    maxPos.x = (atom.position.x > maxPos.x) ? atom.position.x
                                                            : maxPos.x;
                    maxPos.y = (atom.position.y > maxPos.y) ? atom.position.y
                                                            : maxPos.y;
                    maxPos.z = (atom.position.z > maxPos.z) ? atom.position.z
                                                            : maxPos.z;

                    // add Atom to the list
                    atom.processed = 0;
                    if (geometryType == gtSticks ||
                        (geometryType == gtAtomsAndSticks &&
                         atom.residue % 2 == 0))
                        atoms[atom.id] = atom;
                    else
                        atoms[index] = atom;
                }
            }
        }
        file.close();
    }

    vec4f objectSize;
    objectSize.x = (maxPos.x - minPos.x);
    objectSize.y = (maxPos.y - minPos.y);
    objectSize.z = (maxPos.z - minPos.z);

    vec4f center;
    center.x = (minPos.x + maxPos.x) / 2.f;
    center.y = (minPos.y + maxPos.y) / 2.f;
    center.z = (minPos.z + maxPos.z) / 2.f;

    vec4f objectScale;
    objectScale.x = scale.x / (maxPos.x - minPos.x);
    objectScale.y = scale.y / (maxPos.y - minPos.y);
    objectScale.z = scale.z / (maxPos.z - minPos.z);

    float atomDistance(DEFAULT_ATOM_DISTANCE);

    std::map<int, Atom>::iterator it = atoms.begin();
    while (it != atoms.end())
    {
        Atom &atom((*it).second);
        if (atom.processed < 2)
        {
            int nb;

            float radius(atom.position.w);
            float stickRadius(atom.position.w);
            switch (geometryType)
            {
            case gtFixedSizeAtoms:
                radius = defaultAtomSize;
                break;
            case gtSticks:
                radius = defaultStickSize;
                stickRadius = defaultStickSize;
                break;
            case gtAtomsAndSticks:
                radius = atom.position.w / 2.f;
                stickRadius = defaultStickSize / 2.f;
                break;
            case gtBackbone:
                radius = defaultStickSize;
                stickRadius = defaultStickSize;
                break;
            case gtIsoSurface:
                radius = atom.position.w;
                break;
            default:
                break;
            }

            if (geometryType == gtSticks || geometryType == gtAtomsAndSticks ||
                geometryType == gtBackbone)
            {
                std::map<int, Atom>::iterator it2 = atoms.begin();
                while (it2 != atoms.end())
                {
                    if (it2 != it && (*it2).second.processed < 2 &&
                        ((*it).second.isBackbone == (*it2).second.isBackbone))
                    {
                        Atom &atom2((*it2).second);
                        vec4f a;
                        a.x = atom.position.x - atom2.position.x;
                        a.y = atom.position.y - atom2.position.y;
                        a.z = atom.position.z - atom2.position.z;
                        float distance =
                            sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
                        float stickDistance =
                            (geometryType == gtBackbone && atom2.isBackbone)
                                ? DEFAULT_STICK_DISTANCE * 2.f
                                : DEFAULT_STICK_DISTANCE;

                        if (distance < stickDistance)
                        {
                            vec4f halfCenter;
                            halfCenter.x =
                                (atom.position.x + atom2.position.x) / 2.f;
                            halfCenter.y =
                                (atom.position.y + atom2.position.y) / 2.f;
                            halfCenter.z =
                                (atom.position.z + atom2.position.z) / 2.f;

                            // Sticks
                            nb = cudaKernel.addPrimitive(ptCylinder, true);
                            cudaKernel.setPrimitive(
                                nb,
                                objectScale.x * distanceRatio * atomDistance *
                                    (atom.position.x - center.x),
                                objectScale.y * distanceRatio * atomDistance *
                                    (atom.position.y - center.y),
                                objectScale.z * distanceRatio * atomDistance *
                                    (atom.position.z - center.z),
                                objectScale.x * distanceRatio * atomDistance *
                                    (halfCenter.x - center.x),
                                objectScale.y * distanceRatio * atomDistance *
                                    (halfCenter.y - center.y),
                                objectScale.z * distanceRatio * atomDistance *
                                    (halfCenter.z - center.z),
                                objectScale.x * stickRadius, 0.f, 0.f,
                                (geometryType == gtSticks) ? atom.materialId
                                                           : 1010);
                            const vec2f vt0 = make_vec2f(0.f, 0.f);
                            const vec2f vt1 = make_vec2f(1.f, 1.f);
                            const vec2f vt2 = make_vec2f(0.f, 0.f);
                            cudaKernel.setPrimitiveTextureCoordinates(nb, vt0,
                                                                      vt1, vt2);
                        }
                    }
                    it2++;
                }
            }

            bool addAtom(true);

            int m = atom.materialId;
            if (!useModels /*&& (atom.chainId%2==chainSelection)*/)
            {
                // addAtom = false;
                radius = stickRadius;
                if (geometryType == gtAtomsAndSticks)
                    m = 11;
            }

            if (addAtom)
            {
                // Enveloppe
                const vec2f vt0 = make_vec2f(0.f, 0.f);
                const vec2f vt1 = make_vec2f(1.f, 1.f);
                const vec2f vt2 = make_vec2f(0.f, 0.f);
                if (geometryType == gtIsoSurface && atom.isBackbone &&
                    atom.chainId % 2 == 0)
                {
                    nb = cudaKernel.addPrimitive(ptSphere, true);
                    cudaKernel.setPrimitive(
                        nb,
                        objectScale.x * distanceRatio * atomDistance *
                            (atom.position.x - center.x),
                        objectScale.y * distanceRatio * atomDistance *
                            (atom.position.y - center.y),
                        objectScale.z * distanceRatio * atomDistance *
                            (atom.position.z - center.z),
                        objectScale.x * radius * 2.f, 0.f, 0.f, 10);
                    cudaKernel.setPrimitiveTextureCoordinates(nb, vt0, vt1,
                                                              vt2);
                }

                nb = cudaKernel.addPrimitive(ptSphere, true);
                cudaKernel.setPrimitive(
                    nb,
                    objectScale.x * distanceRatio * atomDistance *
                        (atom.position.x - center.x),
                    objectScale.y * distanceRatio * atomDistance *
                        (atom.position.y - center.y),
                    objectScale.z * distanceRatio * atomDistance *
                        (atom.position.z - center.z),
                    objectScale.x * radius, 0.f, 0.f, m);
                cudaKernel.setPrimitiveTextureCoordinates(nb, vt0, vt1, vt2);
            }
        }
        ++it;
    }
    objectSize.x *= objectScale.x * distanceRatio * atomDistance;
    objectSize.y *= objectScale.y * distanceRatio * atomDistance;
    objectSize.z *= objectScale.z * distanceRatio * atomDistance;
    LOG_INFO(1, " - Geometry type...: " << geometryType);
    LOG_INFO(1, " - Number of atoms.: " << atoms.size());
    LOG_INFO(1, " - Object size.....: " << objectSize.x << "," << objectSize.y
                                        << "," << objectSize.z);
    return objectSize;
}
} // namespace solr
