# Bootstrap CPM.cmake (https://github.com/cpm-cmake/CPM.cmake)
# Downloads the package manager once into the build tree (or CPM_SOURCE_CACHE).
set(CPM_DOWNLOAD_VERSION 0.40.2)

if(CPM_SOURCE_CACHE)
  set(CPM_DOWNLOAD_LOCATION "${CPM_SOURCE_CACHE}/cpm/CPM_${CPM_DOWNLOAD_VERSION}.cmake")
elseif(DEFINED ENV{CPM_SOURCE_CACHE})
  set(CPM_DOWNLOAD_LOCATION "$ENV{CPM_SOURCE_CACHE}/cpm/CPM_${CPM_DOWNLOAD_VERSION}.cmake")
else()
  set(CPM_DOWNLOAD_LOCATION "${CMAKE_BINARY_DIR}/cmake/CPM_${CPM_DOWNLOAD_VERSION}.cmake")
endif()

get_filename_component(CPM_DOWNLOAD_DIR "${CPM_DOWNLOAD_LOCATION}" DIRECTORY)
file(MAKE_DIRECTORY "${CPM_DOWNLOAD_DIR}")

if(NOT (EXISTS "${CPM_DOWNLOAD_LOCATION}"))
  message(STATUS "Downloading CPM.cmake v${CPM_DOWNLOAD_VERSION}")
  file(DOWNLOAD
    "https://github.com/cpm-cmake/CPM.cmake/releases/download/v${CPM_DOWNLOAD_VERSION}/CPM.cmake"
    "${CPM_DOWNLOAD_LOCATION}"
    STATUS _cpm_dl_status)
  list(GET _cpm_dl_status 0 _cpm_dl_code)
  if(NOT _cpm_dl_code EQUAL 0)
    message(FATAL_ERROR "Failed to download CPM.cmake: ${_cpm_dl_status}")
  endif()
endif()

include("${CPM_DOWNLOAD_LOCATION}")
