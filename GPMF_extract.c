/*! @file GPMF_demo.c
 *
 *  @brief Demo to extract GPMF from an MP4
 *
 *  @version 2.4.0
 *
 *  (C) Copyright 2017-2020 GoPro Inc (http://gopro.com/).
 *
 *  Licensed under either:
 *  - Apache License, Version 2.0, http://www.apache.org/licenses/LICENSE-2.0
 *  - MIT license, http://opensource.org/licenses/MIT
 *  at your option.
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "GPMF_parser.h"
#include "demo/GPMF_mp4reader.h"

#define	SHOW_THIS_FOUR_CC			STR2FOURCC("GPS5")


void printHelp(char* name)
{
	printf("usage: %s <file_with_GPMF> <optional features>\n", name);
	printf("       -fWXYZ - show only this fourCC , e.g. -f%c%c%c%c (default) just -f for all\n", PRINTF_4CC(SHOW_THIS_FOUR_CC));
	printf("       -h - this help\n");
	printf("       \n");
	printf("       ver 2.0\n");
}

int load_parameters(int argc, char* argv[], char **vide_fn, uint32_t *four_cc) {
  *four_cc = SHOW_THIS_FOUR_CC;

  if (argc < 2) {
    return 1;
  }

  *vide_fn = argv[1];

  for (int i = 2; i < argc; i++) {
    if (argv[i][0] == '-') {    //feature switches
      switch (argv[i][1]) {
        case 'f': *four_cc = STR2FOURCC((&(argv[i][2])));  break;
        case 'h': printHelp(argv[0]);  break;
      }
    }
  }

  return 0;
}

int main(int argc, char* argv[])
{
	uint32_t show_this_four_cc;
  char *video_fn;
  if(load_parameters(argc, argv, &video_fn, &show_this_four_cc)) {
    printHelp(argv[0]);
    return EXIT_FAILURE;
  }

	size_t mp4handle = OpenMP4Source(argv[1], MOV_GPMF_TRAK_TYPE, MOV_GPMF_TRAK_SUBTYPE, 0);
	if (mp4handle == 0) {
		printf("error: %s is an invalid MP4/MOV or it has no GPMF data\n\n", argv[1]);
		return EXIT_FAILURE;
	}


  GPMF_ERR ret = GPMF_OK;
	double metadatalength = GetDuration(mp4handle);
	if (metadatalength < 0.01) {
    return EXIT_FAILURE;
  }

  uint32_t payloads = GetNumberPayloads(mp4handle);
  //		printf("found %.2fs of metadata, from %d payloads, within %s\n", metadatalength, payloads, argv[1]);

  uint32_t fr_num, fr_dem;
  uint32_t frames = GetVideoFrameRateAndCount(mp4handle, &fr_num, &fr_dem);

  size_t payloadres;
  GPMF_stream metadata_stream, * ms = &metadata_stream;
  for (uint32_t index = 0; index < payloads; index++)
  {
    uint32_t payloadsize = GetPayloadSize(mp4handle, index);
    payloadres = GetPayloadResource(mp4handle, payloadres, payloadsize);
    uint32_t* payload = GetPayload(mp4handle, payloadres, index);
    if (payload == NULL)
      break;

    double in = 0.0, out = 0.0; //times
    ret = GetPayloadTime(mp4handle, index, &in, &out);
    if (ret != GPMF_OK)
      break;

    ret = GPMF_Init(ms, payload, payloadsize);
    if (ret != GPMF_OK)
      break;

    while (GPMF_OK == GPMF_FindNext(ms, STR2FOURCC("STRM"), GPMF_RECURSE_LEVELS|GPMF_TOLERANT)) {   //GoPro Hero5/6/7 Accelerometer)

      if (GPMF_VALID_FOURCC(show_this_four_cc))
      {
        if (GPMF_OK != GPMF_FindNext(ms, show_this_four_cc, GPMF_RECURSE_LEVELS|GPMF_TOLERANT))
          continue;
      }
      else
      {
        ret = GPMF_SeekToSamples(ms);
        if (GPMF_OK != ret)
          continue;
      }

      char* rawdata = (char*)GPMF_RawData(ms);
      uint32_t key = GPMF_Key(ms);
      GPMF_SampleType type = GPMF_Type(ms);
      uint32_t samples = GPMF_Repeat(ms);
      uint32_t elements = GPMF_ElementsInStruct(ms);

      if (samples)
      {
        uint32_t buffersize = samples * elements * sizeof(double);
        GPMF_stream find_stream;
        double* ptr, * tmpbuffer = (double*)malloc(buffersize);

#define MAX_UNITS	64
#define MAX_UNITLEN	8
        char units[MAX_UNITS][MAX_UNITLEN] = { "" };
        uint32_t unit_samples = 1;

        char complextype[MAX_UNITS] = { "" };
        uint32_t type_samples = 1;

        if (tmpbuffer)
        {
          uint32_t i, j;

          //Search for any units to display
          GPMF_CopyState(ms, &find_stream);
          if (GPMF_OK == GPMF_FindPrev(&find_stream, GPMF_KEY_SI_UNITS, GPMF_CURRENT_LEVEL | GPMF_TOLERANT) ||
            GPMF_OK == GPMF_FindPrev(&find_stream, GPMF_KEY_UNITS, GPMF_CURRENT_LEVEL | GPMF_TOLERANT))
          {
            char* data = (char*)GPMF_RawData(&find_stream);
            uint32_t ssize = GPMF_StructSize(&find_stream);
            if (ssize > MAX_UNITLEN - 1) ssize = MAX_UNITLEN - 1;
            unit_samples = GPMF_Repeat(&find_stream);

            for (i = 0; i < unit_samples && i < MAX_UNITS; i++)
            {
              memcpy(units[i], data, ssize);
              units[i][ssize] = 0;
              data += ssize;
            }
          }

          //Search for TYPE if Complex
          GPMF_CopyState(ms, &find_stream);
          type_samples = 0;
          if (GPMF_OK == GPMF_FindPrev(&find_stream, GPMF_KEY_TYPE, GPMF_CURRENT_LEVEL | GPMF_TOLERANT))
          {
            char* data = (char*)GPMF_RawData(&find_stream);
            uint32_t ssize = GPMF_StructSize(&find_stream);
            if (ssize > MAX_UNITLEN - 1) ssize = MAX_UNITLEN - 1;
            type_samples = GPMF_Repeat(&find_stream);

            for (i = 0; i < type_samples && i < MAX_UNITS; i++)
            {
              complextype[i] = data[i];
            }
          }

          //GPMF_FormattedData(ms, tmpbuffer, buffersize, 0, samples); // Output data in LittleEnd, but no scale
          if (GPMF_OK == GPMF_ScaledData(ms, tmpbuffer, buffersize, 0, samples, GPMF_TYPE_DOUBLE))//Output scaled data as floats
          {

            ptr = tmpbuffer;
            int pos = 0;
            for (i = 0; i < samples; i++)
            {
              printf("  %c%c%c%c ", PRINTF_4CC(key));

              for (j = 0; j < elements; j++)
              {
                if (type == GPMF_TYPE_STRING_ASCII)
                {
                  printf("%c", rawdata[pos]);
                  pos++;
                  ptr++;
                }
                else if (type_samples == 0) //no TYPE structure
                  printf("%.3f%s, ", *ptr++, units[j % unit_samples]);
                else if (complextype[j] != 'F')
                {
                  printf("%.3f%s, ", *ptr++, units[j % unit_samples]);
                  pos += GPMF_SizeofType((GPMF_SampleType)complextype[j]);
                }
                else if (type_samples && complextype[j] == GPMF_TYPE_FOURCC)
                {
                  ptr++;
                  printf("%c%c%c%c, ", rawdata[pos], rawdata[pos + 1], rawdata[pos + 2], rawdata[pos + 3]);
                  pos += GPMF_SizeofType((GPMF_SampleType)complextype[j]);
                }
              }


              printf("\n");
            }
          }
          free(tmpbuffer);
        }
      }
    }
    GPMF_ResetState(ms);
  }

  if (payloadres) {
    FreePayloadResource(mp4handle, payloadres);
  }
  if (ms) {
    GPMF_Free(ms);
  }
  CloseSource(mp4handle);

	if (ret != 0) {
		if (GPMF_ERROR_UNKNOWN_TYPE == ret)
			printf("Unknown GPMF Type within\n");
		else
			printf("GPMF data has corruption\n");
	}

	return (int)ret;
}
