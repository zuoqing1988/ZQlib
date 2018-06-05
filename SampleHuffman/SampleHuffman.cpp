#include "ZQ_Huffman.h"

using namespace ZQ;
int main()
{
	const char* input = "1234567890";
	char* output;
	unsigned long outlen;
	char* input2;
	unsigned long inlen2;
	ZQ_HuffmanEndec::ZQ_HuffmanEncodeByteStream((const unsigned char*)input,strlen(input)+1,(unsigned char**)&output,&outlen);
	ZQ_HuffmanEndec::ZQ_HuffmanDecodeByteStream((const unsigned char*)output,outlen,(unsigned char**)&input2,&inlen2);
	puts(input);
	puts(input2);


	char name[20];
	ZQ_HuffmanEndec::ZQ_HuffmanEncodeFile("input.txt","input.enc");
	ZQ_HuffmanEndec::ZQ_HuffmanDecodeFile("input.enc","output.txt",name);
	puts(name);

	return 0;
}