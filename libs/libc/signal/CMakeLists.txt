# ##############################################################################
# libs/libc/signal/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License.  You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations under
# the License.
#
# ##############################################################################

target_sources(
  c
  PRIVATE sig_addset.c
          sig_delset.c
          sig_emptyset.c
          sig_fillset.c
          sig_nandset.c
          sig_andset.c
          sig_orset.c
          sig_xorset.c
          sig_isemptyset.c
          sig_killpg.c
          sig_altstack.c
          sig_interrupt.c
          sig_hold.c
          sig_ignore.c
          sig_ismember.c
          sig_pause.c
          sig_psignal.c
          sig_raise.c
          sig_relse.c
          sig_set.c
          sig_signal.c
          sig_wait.c)
