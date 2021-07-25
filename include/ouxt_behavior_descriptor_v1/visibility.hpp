// Copyright (c) 2021, OUXT-Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OUXT_BEHAVIOR_DESCRIPTOR_V1__VISIBILITY_HPP_
#define OUXT_BEHAVIOR_DESCRIPTOR_V1__VISIBILITY_HPP_

#ifdef __cplusplus
extern "C" {
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__

#ifdef __GNUC__
#define OUXT_BEHAVIOR_DESCRIPTOR_V1__EXPORT __attribute__((dllexport))
#define OUXT_BEHAVIOR_DESCRIPTOR_V1__IMPORT __attribute__((dllimport))
#else
#define OUXT_BEHAVIOR_DESCRIPTOR_V1__EXPORT __declspec(dllexport)
#define OUXT_BEHAVIOR_DESCRIPTOR_V1__IMPORT __declspec(dllimport)
#endif

#ifdef OUXT_BEHAVIOR_DESCRIPTOR_V1__DLL
#define OUXT_BEHAVIOR_DESCRIPTOR_V1__PUBLIC OUXT_BEHAVIOR_DESCRIPTOR_V1__EXPORT
#else
#define OUXT_BEHAVIOR_DESCRIPTOR_V1__PUBLIC OUXT_BEHAVIOR_DESCRIPTOR_V1__IMPORT
#endif

#define OUXT_BEHAVIOR_DESCRIPTOR_V1__PUBLIC_TYPE \
  OUXT_BEHAVIOR_DESCRIPTOR_V1__PUBLIC

#define OUXT_BEHAVIOR_DESCRIPTOR_V1__LOCAL

#else

#define OUXT_BEHAVIOR_DESCRIPTOR_V1__EXPORT \
  __attribute__((visibility("default")))
#define OUXT_BEHAVIOR_DESCRIPTOR_V1__IMPORT

#if __GNUC__ >= 4
#define OUXT_BEHAVIOR_DESCRIPTOR_V1__PUBLIC \
  __attribute__((visibility("default")))
#define OUXT_BEHAVIOR_DESCRIPTOR_V1__LOCAL __attribute__((visibility("hidden")))
#else
#define OUXT_BEHAVIOR_DESCRIPTOR_V1__PUBLIC
#define OUXT_BEHAVIOR_DESCRIPTOR_V1__LOCAL
#endif

#define OUXT_BEHAVIOR_DESCRIPTOR_V1__PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // OUXT_BEHAVIOR_DESCRIPTOR_V1__VISIBILITY_HPP_
