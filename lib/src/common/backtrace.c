/**
 * Copyright 2016-2021 ETH Zurich
 *
 * This file is part of dlProbe, a modified version of srsRAN.
 *
 * srsRAN is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsRAN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */
#include <execinfo.h>
#include <stdio.h>
#include <stdlib.h>

void srsran_backtrace_print(FILE* f)
{
  void* array[128] = {};
  int   size       = 0;

  if (f) {
    // Get all stack traces
    size = backtrace(array, 128);

    char** symbols = backtrace_symbols(array, size);

    for (int i = 1; i < size; i++) {
      fprintf(f, "\t%s\n", symbols[i]);
    }
    free(symbols);
  }
}
