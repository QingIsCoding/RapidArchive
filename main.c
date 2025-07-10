#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "compress_decompress.h"

int main() {
    // 使用重复度较高的长字符串
    const char* originalData = 
        "This is a long string that contains a lot of repeated content. "
        "This is a long string that contains a lot of repeated content. "
        "This is a long string that contains a lot of repeated content. "
        "This is a long string that contains a lot of repeated content. "
        "This is a long string that contains a lot of repeated content. "
        "This is a long string that contains a lot of repeated content. "
        "This is a long string that contains a lot of repeated content. "
        "This is a long string that contains a lot of repeated content. "
        "This is a long string that contains a lot of repeated content. "
        "This is a long string that contains a lot of repeated content. "
        "This is a long string that contains a lot of repeated content. "
        "This is a long string that contains a lot of repeated content. "
        "This is a long string that contains a lot of repeated content. "
        "This is a long string that contains a lot of repeated content. "
        "This is a long string that contains a lot of repeated content. "
        "This is a long string that contains a lot of repeated content. "
        "This is a long string that contains a lot of repeated content. "
        "This is a long string that contains a lot of repeated content. "
        "This is a long string that contains a lot of repeated content. "
        "This is a long string that contains a lot of repeated content. "
        "This is a long string that contains a lot of repeated content. "
        "This is a long string that contains a lot of repeated content. "
        "This is a long string that contains a lot of repeated content. "
        "This is a long string that contains a lot of repeated content. "
        "This is a long string that contains a lot of repeated content. "
        "This is a long string that contains a lot of repeated content. "
        "This is a long string that contains a lot of repeated content. "
        "This is a long string that contains a lot of repeated content. "
        "This is a long string that contains a lot of repeated content. "
        "This is a long string that contains a lot of repeated content. "
        "This is a long string that contains a lot of repeated content. ";

    int originalLength = strlen(originalData) + 1; // 包括字符串结束符
    char* compressedData = malloc(originalLength + 64); // 预留一些空间用于压缩
    if (compressedData == NULL) {
        printf("Memory allocation failed!\n");
        return 1;
    }
    // 压缩数据
    int compressedLength = fastlz_compress_level(1, originalData, originalLength, compressedData);
    if (compressedLength > 0) {
        printf("Original size: %d bytes\n", originalLength);
        printf("Compressed size: %d bytes\n", compressedLength);
        
        // 输出压缩后的数据（以十六进制格式显示）
        printf("Compressed data (hex): ");
        for (int i = 0; i < compressedLength; i++) {
            printf("%02x ", (unsigned char)compressedData[i]);
        }
        printf("\n");
    } else {
        printf("Compression failed!\n");
    }

    return 0;
}