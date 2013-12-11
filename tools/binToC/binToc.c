
/* \file   binToc.c
** 
** \brief  This tool will convert a binary file to 'C' header file.
*/


/*
* Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
*/
/*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

 
#include<stdio.h>
#include<stdlib.h>

int main(int argc,char *argv[])
{
	FILE			*sourceFile, *destFile;
	unsigned int 	lineLength = 0;
	unsigned int 	fileSize = 0;	
	signed char 	byteDate = 0;	

	if(argc != 3)
	{
		printf("Please execute the file in this format\n");
		printf("./a.exe  <bin file name> <output file name>\n");
		exit(1);
	}

	sourceFile =(FILE* ) fopen(argv[1],"rb");
	destFile =(FILE* ) fopen(argv[2],"w");
	
	fseek(sourceFile, 0L, SEEK_END);
	fileSize = ftell(sourceFile);
	fseek(sourceFile, 0L, SEEK_SET);

	fprintf(destFile,"%s\n","unsigned char const image1[] = {");
	while(fileSize--)
    {
        byteDate = fgetc(sourceFile);
	
		if(((unsigned char)(byteDate)) < 16)
			fprintf(destFile,"0x0%X", (unsigned char)byteDate);	
		else
			fprintf(destFile,"0x%X", (unsigned char)byteDate);

		fprintf(destFile,"%c ", ',');
		
		lineLength++;
		if(lineLength == 10)
		{
			fprintf(destFile,"%c", '\n');
			lineLength = 0;
		}			
    }
	fprintf(destFile, "%s", "};");
	
	/* Close the files */
	close(sourceFile);
	close(destFile);
	
	return 0;

}

