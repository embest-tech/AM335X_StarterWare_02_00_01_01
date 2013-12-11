/**
 * makefsdata: Converts a directory structure for use with the lwIP httpd.
 *
 * This file is part of the lwIP TCP/IP stack.
 * 
 * Author: Jim Pettinato
 *         Simon Goldschmidt
 *
 */

#include <stdio.h>
#include <stdlib.h>
#ifdef WIN32
#include "windows.h"
#else
#include <dir.h>
#endif
#include <dos.h>
#include <string.h>

/* Compatibility defines Win32 vs. DOS */
#ifdef WIN32

#define FIND_T                        WIN32_FIND_DATAA
#define FIND_T_FILENAME(fInfo)        (fInfo.cFileName)
#define FIND_T_IS_DIR(fInfo)          ((fInfo.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0)
#define FIND_T_IS_FILE(fInfo)         ((fInfo.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) == 0)
#define FIND_RET_T                    HANDLE
#define FINDFIRST_FILE(path, result)  FindFirstFileA(path, result)
#define FINDFIRST_DIR(path, result)   FindFirstFileA(path, result)
#define FINDNEXT(ff_res, result)      FindNextFileA(ff_res, result)
#define FINDFIRST_SUCCEEDED(ret)      (ret != INVALID_HANDLE_VALUE)
#define FINDNEXT_SUCCEEDED(ret)       (ret == TRUE)

#define GETCWD(path, len)             GetCurrentDirectoryA(len, path)
#define CHDIR(path)                   SetCurrentDirectoryA(path)

#define NEWLINE "\r\n"

#else

#define FIND_T                        struct fflbk
#define FIND_T_FILENAME(fInfo)        (fInfo.ff_name)
#define FIND_T_IS_DIR(fInfo)          ((fInfo.ff_attrib & FA_DIREC) == FA_DIREC)
#define FIND_T_IS_FILE(fInfo)         (1)
#define FIND_RET_T                    int
#define FINDFIRST_FILE(path, result)  findfirst(path, result, FA_ARCH)
#define FINDFIRST_DIR(path, result)   findfirst(path, result, FA_DIREC)
#define FINDNEXT(ff_res, result)      FindNextFileA(ff_res, result)
#define FINDFIRST_SUCCEEDED(ret)      (ret == 0)
#define FINDNEXT_SUCCEEDED(ret)       (ret == 0)

#define GETCWD(path, len)             getcwd(path, len)
#define CHDIR(path)                   chdir(path)

#endif

typedef unsigned char u8_t;
#include "../httpd.h"

/** (Your server name here) */
const char *serverID =
"Server: lwIP/1.3.1 (http://savannah.nongnu.org/projects/lwip)\r\n";

/* change this to suit your MEM_ALIGNMENT */
#define PAYLOAD_ALIGNMENT 4
/* set this to 0 to prevent aligning payload */
#define ALIGN_PAYLOAD 1
/* define this to a type that has the required alignment */
#define PAYLOAD_ALIGN_TYPE "unsigned int"
static int payload_alingment_dummy_counter = 0;

#define HEX_BYTES_PER_LINE 16

#define MAX_PATH_LEN 256

int process_sub(FILE *data_file, FILE *struct_file);
int process_file(FILE *data_file, FILE *struct_file, const char *filename);
int file_write_http_header(FILE *data_file, const char *filename, int file_size);
int file_put_ascii(FILE *file, const char* ascii_string, int len, int *i);

char curSubdir[MAX_PATH_LEN];
char lastFileVar[MAX_PATH_LEN];

unsigned char processSubs = 1;
unsigned char includeHttpHeader = 1;
unsigned char useHttp11 = 0;

int main(int argc, char *argv[])
{
  FIND_T fInfo;
  FIND_RET_T fret;
  char path[MAX_PATH_LEN];
  char appPath[MAX_PATH_LEN];
  FILE *data_file;
  FILE *struct_file;
  int filesProcessed;
  int i;

  memset(path, 0, sizeof(path));
  memset(appPath, 0, sizeof(appPath));

  printf(NEWLINE " makefsdata - HTML to C source converter" NEWLINE);
  printf("     by Jim Pettinato               - circa 2003 " NEWLINE);
  printf("     extended by Simon Goldschmidt  - 2009 " NEWLINE NEWLINE);

  strcpy(path, "fs");
  for(i = 1; i < argc; i++) {
    if (strstr(argv[i], "-s")) {
      processSubs = 0;
    } else if (strstr(argv[i], "-e")) {
      includeHttpHeader = 0;
    } else if (strstr(argv[i], "-11")) {
      useHttp11 = 1;
    } else {
      strcpy(path, argv[i]);
    }
  }

  /* if command line param or subdir named 'fs' not found spout usage verbiage */
  fret = FINDFIRST_DIR(path, &fInfo);
  if (!FINDFIRST_SUCCEEDED(fret)) {
    /* if no subdir named 'fs' (or the one which was given) exists, spout usage verbiage */
    printf(" Failed to open directory \"%s\"." NEWLINE NEWLINE, path);
    printf(" Usage: htmlgen [targetdir] [-s] [-i]" NEWLINE NEWLINE);
    printf("   targetdir: relative or absolute path to files to convert" NEWLINE);
    printf("   switch -s: toggle processing of subdirectories (default is on)" NEWLINE);
    printf("   switch -e: exclude HTTP header from file (header is created at runtime, default is on)" NEWLINE);
    printf("   switch -11: include HTTP 1.1 header (1.0 is default)" NEWLINE);
    printf("   if targetdir not specified, htmlgen will attempt to" NEWLINE);
    printf("   process files in subdirectory 'fs'" NEWLINE);
    exit(-1);
  }

  printf("HTTP %sheader will %s statically included." NEWLINE,
    (includeHttpHeader ? (useHttp11 ? "1.1 " : "1.0 ") : ""),
    (includeHttpHeader ? "be" : "not be"));

  sprintf(curSubdir, "");  /* start off in web page's root directory - relative paths */
  printf("  Processing all files in directory %s", path);
  if (processSubs) {
    printf(" and subdirectories..." NEWLINE NEWLINE);
  } else {
    printf("..." NEWLINE NEWLINE);
  }

  GETCWD(appPath, MAX_PATH_LEN);
  data_file = fopen("fsdata.tmp", "wb");
  struct_file = fopen("fshdr.tmp", "wb");

  CHDIR(path);

  fprintf(data_file, "#include \"fs.h\"" NEWLINE);
  fprintf(data_file, "#include \"lwip\\def.h\"" NEWLINE);
  fprintf(data_file, "#include \"fsdata.h\"" NEWLINE NEWLINE NEWLINE);

  fprintf(data_file, "#define file_NULL (struct fsdata_file *) NULL" NEWLINE NEWLINE NEWLINE);

  sprintf(lastFileVar, "NULL");

  filesProcessed = process_sub(data_file, struct_file);

  /* data_file now contains all of the raw data.. now append linked list of
   * file header structs to allow embedded app to search for a file name */
  fprintf(data_file, NEWLINE NEWLINE);
  fprintf(struct_file, "#define FS_ROOT file_%s" NEWLINE, lastFileVar);
  fprintf(struct_file, "#define FS_NUMFILES %d" NEWLINE NEWLINE, filesProcessed);

  fclose(data_file);
  fclose(struct_file);

  CHDIR(appPath);
  /* append struct_file to data_file */
  system("copy /b /y fsdata.tmp+fshdr.tmp fsdata.c");

  printf(NEWLINE "Processed %d files - done." NEWLINE NEWLINE, filesProcessed);

  return 0;
}


int process_sub(FILE *data_file, FILE *struct_file)
{
  FIND_T fInfo;
  FIND_RET_T fret;
  int filesProcessed = 0;
  char oldSubdir[MAX_PATH_LEN];

  if (processSubs) {
    /* process subs recursively */
    strcpy(oldSubdir, curSubdir);
    fret = FINDFIRST_DIR("*", &fInfo);
    if (FINDFIRST_SUCCEEDED(fret)) {
      do {
        const char *curName = FIND_T_FILENAME(fInfo);
        if (strcmp(curName, ".") == 0) continue;
        if (strcmp(curName, "..") == 0) continue;
        if (strcmp(curName, "CVS") == 0) continue;
        if (!FIND_T_IS_DIR(fInfo)) continue;
        CHDIR(curName);
        strcat(curSubdir, "/");
        strcat(curSubdir, curName);
        printf(NEWLINE "processing subdirectory %s/..." NEWLINE, curSubdir);
        filesProcessed += process_sub(data_file, struct_file);
        CHDIR("..");
        strcpy(curSubdir, oldSubdir);
      } while (FINDNEXT_SUCCEEDED(FINDNEXT(fret, &fInfo)));
    }
  }

  fret = FINDFIRST_FILE("*.*", &fInfo);
  if (FINDFIRST_SUCCEEDED(fret)) {
    /* at least one file in directory */
    do {
      if (FIND_T_IS_FILE(fInfo)) {
        const char *curName = FIND_T_FILENAME(fInfo);
        printf("processing %s/%s..." NEWLINE, curSubdir, curName);
        if (process_file(data_file, struct_file, curName) < 0) {
          printf(NEWLINE "Error... aborting" NEWLINE);
          return -1;
        }
        filesProcessed++;
      }
    } while (FINDNEXT_SUCCEEDED(FINDNEXT(fret, &fInfo)));
  }
  return filesProcessed;
}

int get_file_size(const char* filename)
{
  FILE *inFile;
  int file_size = -1;
  inFile = fopen(filename, "rb");
  if(inFile) {
    fseek (inFile , 0 , SEEK_END);
    file_size = ftell(inFile);
    fclose(inFile);
  }
  return file_size;
}

int process_file(FILE *data_file, FILE *struct_file, const char *filename)
{
  char *pch;
  char varname[MAX_PATH_LEN];
  int i = 0;
  int ch;
  FILE *sourceFile;
  char qualifiedName[MAX_PATH_LEN];
  int file_size;

  /* create qualified name (TODO: prepend slash or not?) */
  sprintf(qualifiedName,"%s/%s", curSubdir, filename);
  /* create C variable name */
  strcpy(varname, qualifiedName);
  /* convert slashes & dots to underscores */
  while ((pch = strpbrk(varname, "./\\")) != NULL) {
    *pch = '_';
  }
#if ALIGN_PAYLOAD
  /* to force even alignment of array */
  fprintf(data_file, "static const " PAYLOAD_ALIGN_TYPE " dummy_align_%s = %d;" NEWLINE, varname, payload_alingment_dummy_counter++);
#endif /* ALIGN_PAYLOAD */
  fprintf(data_file, "static const unsigned char data_%s[] = {" NEWLINE, varname);
  /* encode source file name (used by file system, not returned to browser) */
  fprintf(data_file, "/* %s (%d chars) */" NEWLINE, qualifiedName, strlen(qualifiedName)+1);
  file_put_ascii(data_file, qualifiedName, strlen(qualifiedName)+1, &i);
#if ALIGN_PAYLOAD
  /* pad to even number of bytes to assure payload is on aligned boundary */
  while(i % PAYLOAD_ALIGNMENT != 0) {
    fprintf(data_file, "0x%02.2x,", 0);
    i++;
  }
#endif /* ALIGN_PAYLOAD */
  fprintf(data_file, NEWLINE);

  /* build declaration of struct fsdata_file in temp file */
  fprintf(struct_file, "const struct fsdata_file file_%s[] = { {" NEWLINE, varname);
  fprintf(struct_file, "file_%s," NEWLINE, lastFileVar);
  fprintf(struct_file, "data_%s," NEWLINE, varname);
  fprintf(struct_file, "data_%s + %d," NEWLINE, varname, i);
  fprintf(struct_file, "sizeof(data_%s) - %d," NEWLINE, varname, i);
  fprintf(struct_file, "%d}};" NEWLINE NEWLINE, includeHttpHeader);
  strcpy(lastFileVar, varname);

  file_size = get_file_size(filename);
  if (includeHttpHeader) {
    file_write_http_header(data_file, filename, file_size);
  }
  /* write actual file contents */
  i = 0;
  fprintf(data_file, NEWLINE "/* raw file data (%d bytes) */" NEWLINE, file_size);
  sourceFile = fopen(filename, "rb");
  while ((ch = fgetc(sourceFile)) != EOF) {
    char c = (char)ch;
    file_put_ascii(data_file, &c, 1, &i);
  }
  fprintf(data_file, "};" NEWLINE NEWLINE);

  return 0;
}

int file_write_http_header(FILE *data_file, const char *filename, int file_size)
{
  int i = 0;
  enum getResponseEnum response_type = HTTP_200_OK;
  enum file_type_enum file_type;
  const char **httpResponseText = httpResponseText_1_0;
  const char *cur_string;
  size_t cur_len;
  int written = 0;
  
  if (useHttp11) {
    httpResponseText = httpResponseText_1_1;
  }

  fprintf(data_file, NEWLINE "/* HTTP header */");
  if (strstr(filename, "404")) {
    response_type = HTTP_404_NOT_FOUND;
  }
  cur_string = httpResponseText[response_type];
  cur_len = strlen(cur_string);
  fprintf(data_file, NEWLINE "/* \"%s\" (%d bytes) */" NEWLINE, cur_string, cur_len);
  written += file_put_ascii(data_file, cur_string, cur_len, &i);
  i = 0;

  cur_string = serverID;
  cur_len = strlen(cur_string);
  fprintf(data_file, NEWLINE "/* \"%s\" (%d bytes) */" NEWLINE, cur_string, cur_len);
  written += file_put_ascii(data_file, cur_string, cur_len, &i);
  i = 0;

  if (strstr(filename, ".html") || strstr(filename, ".htm")) {
    file_type = HTML;
  } else if (strstr(filename, ".gif")) {
    file_type = GIF;
  } else if (strstr(filename, ".png")) {
    file_type = PNG;
  } else if (strstr(filename, ".jpeg") || strstr(filename, ".jpg")) {
    file_type = JPEG;
  } else if (strstr(filename, ".bin") || strstr(filename, ".class")) {
    file_type = OCTET_STREAM;
  } else if (strstr(filename, ".ra") || strstr(filename, ".ram")) {
    file_type = REALAUDIO;
  } else if (strstr(filename, ".js")) {
    file_type = JAVASCRIPT;
  } else if (strstr(filename, ".css")) {
    file_type = CSS;
  } else {
    file_type = TEXT;
  }
  cur_string = httpContentType_header[file_type];
  cur_len = strlen(cur_string);
  fprintf(data_file, NEWLINE "/* \"%s\" (%d bytes) */" NEWLINE, cur_string, cur_len);
  written += file_put_ascii(data_file, cur_string, cur_len, &i);
  i = 0;

  if (useHttp11) {
    char intbuf[MAX_PATH_LEN];
    memset(intbuf, 0, sizeof(intbuf));

    cur_string = httpContentLength;
    cur_len = strlen(cur_string);
    fprintf(data_file, NEWLINE "/* \"%s%d\r\n\" (%d+ bytes) */" NEWLINE, cur_string, file_size, cur_len+2);
    written += file_put_ascii(data_file, cur_string, cur_len, &i);
    _itoa(file_size, intbuf, 10);
    written += file_put_ascii(data_file, intbuf, strlen(intbuf), &i);
    written += file_put_ascii(data_file, "\r\n", 2, &i);
    i = 0;

    cur_string = httpConnectionClose;
    cur_len = strlen(cur_string);
    fprintf(data_file, NEWLINE "/* \"%s\" (%d bytes) */" NEWLINE, cur_string, cur_len);
    written += file_put_ascii(data_file, cur_string, cur_len, &i);
  }

    fprintf(data_file, NEWLINE "/* Empty line (end of header - 2 bytes) */" NEWLINE, cur_string);
  written += file_put_ascii(data_file, "\r\n", 2, &i);

  return written;
}

int file_put_ascii(FILE *file, const char* ascii_string, int len, int *i)
{
  int x;
  for(x = 0; x < len; x++) {
    unsigned char cur = ascii_string[x];
    fprintf(file, "0x%02.2x,", cur);
    if ((++(*i) % HEX_BYTES_PER_LINE) == 0) {
      fprintf(file, NEWLINE);
    }
  }
  return len;
}
