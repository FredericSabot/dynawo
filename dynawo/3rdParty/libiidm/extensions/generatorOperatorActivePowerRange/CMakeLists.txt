# Copyright (c) 2016-2019, RTE (http://www.rte-france.com)
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.

add_subdirectory(core)

if( BUILD_XML )
  add_subdirectory(xml)
endif()

if( BUILD_SAMPLES )
  add_subdirectory(samples)
endif()

# propagate the definition to parent scope
set(CPACK_COMPONENTS_ALL ${CPACK_COMPONENTS_ALL} PARENT_SCOPE)
