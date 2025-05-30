// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "Device/Driver/LX/Convert.hpp"
#include "io/BufferedOutputStream.hxx"
#include "io/FileOutputStream.hxx"
#include "util/PrintException.hxx"
#include "TestUtil.hpp"

#include <memory>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const long MAX_LXN_SIZE = 1024 * 1024;

static const char *lxn_path = "test/data/lxn_to_igc/18BF14K1.FIL";
static const char *igc_in_path = "test/data/lxn_to_igc/18BF14K1.igc";
static const char *igc_out_path = "output/18BF14K1.igc";

static bool
RunConversion()
{
  FILE *lxn_file = fopen(lxn_path, "rb");
  if (lxn_file == NULL) {
    fprintf(stderr, "Failed to open file %s\n", lxn_path);
    return false;
  }

  FileOutputStream igc_fos((const Path) Path(igc_out_path));
  BufferedOutputStream igc_bos(igc_fos);

  long size;
  if (fseek(lxn_file, 0, SEEK_END) != 0 || (size = ftell(lxn_file)) <= 0 ||
      fseek(lxn_file, 0, SEEK_SET) != 0 || size > MAX_LXN_SIZE)  {
    fprintf(stderr, "Failed to seek file %s\n", lxn_path);
    fclose(lxn_file);
    return false;
  }

  void *data = malloc(size);
  size_t n = fread(data, 1, size, lxn_file);
  fclose(lxn_file);
  if (n != (size_t)size) {
    free(data);
    fprintf(stderr, "Failed to read from file %s\n", lxn_path);
    return false;
  }

  bool success = ok1(LX::ConvertLXNToIGC(data, n, igc_bos));
  free(data);

  igc_bos.Flush();
  igc_fos.Commit();
  return success;
}

static bool
CompareFiles()
{
  FILE *igc_in_file = fopen(igc_in_path, "rb");
  if (igc_in_file == NULL) {
    fprintf(stderr, "Failed to open file %s\n", igc_in_path);
    return false;
  }

  FILE *igc_out_file = fopen(igc_out_path, "rb");
  if (igc_out_file == NULL) {
    fprintf(stderr, "Failed to open file %s\n", igc_out_path);
    fclose(igc_in_file);
    return false;
  }

  long in_size;
  if (fseek(igc_in_file, 0, SEEK_END) != 0 || (in_size = ftell(igc_in_file)) <= 0 ||
      fseek(igc_in_file, 0, SEEK_SET) != 0 || in_size > MAX_LXN_SIZE)  {
    fprintf(stderr, "Failed to seek file %s\n", igc_in_path);
    fclose(igc_in_file);
    fclose(igc_out_file);
    return false;
  }

  long out_size;
  if (fseek(igc_out_file, 0, SEEK_END) != 0 || (out_size = ftell(igc_out_file)) <= 0 ||
      fseek(igc_out_file, 0, SEEK_SET) != 0 || out_size > MAX_LXN_SIZE)  {
    fprintf(stderr, "Failed to seek file %s\n", igc_out_path);
    fclose(igc_in_file);
    fclose(igc_out_file);
    return false;
  }

  if (in_size != out_size) {
    fprintf(stderr, "File size doesn't match\n");
    fclose(igc_in_file);
    fclose(igc_out_file);
    return false;
  }

  const auto in_data = std::make_unique<std::byte[]>(in_size);
  size_t in_n = fread(in_data.get(), 1, in_size, igc_in_file);
  fclose(igc_in_file);
  if (in_n != (size_t)in_size) {
    fprintf(stderr, "Failed to read from file %s\n", igc_in_path);
    fclose(igc_out_file);
    return false;
  }

  const auto out_data = std::make_unique<std::byte[]>(out_size);
  size_t out_n = fread(out_data.get(), 1, out_size, igc_out_file);
  fclose(igc_out_file);
  if (out_n != (size_t)in_size) {
    fprintf(stderr, "Failed to read from file %s\n", igc_out_path);
    return false;
  }

  return memcmp(in_data.get(), out_data.get(), in_size) == 0;
}

int main()
try {
  plan_tests(2);

  if (!RunConversion())
    skip(1, 0, "conversion failed");

  ok1(CompareFiles());

  return exit_status();
} catch (...) {
  PrintException(std::current_exception());
  return EXIT_FAILURE;
}
