# Copyright (c) 2011-2022, Cyrille Favreau
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

FIND_PATH(QUANTIS_INCLUDE_DIR Quantis.h
	${CMAKE_INSTALL_PREFIX}/include
    /usr/include
    /opt/include
)

FIND_LIBRARY(QUANTIS_LIBRARY 
    NAMES Quantis
    PATHS
	${CMAKE_INSTALL_PREFIX}/lib64
    /usr/local/lib
    /usr/lib
    /opt/lib
)

SET(QUANTIS_FOUND "NO")
IF(QUANTIS_LIBRARY AND QUANTIS_INCLUDE_DIR)
    SET(QUANTIS_FOUND "YES")
ENDIF(QUANTIS_LIBRARY AND QUANTIS_INCLUDE_DIR)