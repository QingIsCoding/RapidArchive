#ifndef FASTLZ_H
#define FASTLZ_H

#define FASTLZ_VERSION 0x000500

#define FASTLZ_VERSION_MAJOR 0
#define FASTLZ_VERSION_MINOR 5
#define FASTLZ_VERSION_REVISION 0

#define FASTLZ_VERSION_STRING "0.5.0"
/**
压缩输入缓冲区中的数据块，并返回压缩块的大小。输入缓冲区的大小由参数 length 指定。最小输入缓冲区大小为 16 字节。


输出缓冲区必须比输入缓冲区大至少 5%，并且不能小于 66 字节。


如果输入数据不可压缩，则返回值可能会大于 length（输入缓冲区大小）。


输入缓冲区和输出缓冲区不能重叠。


压缩级别可以通过参数 level 指定。目前仅支持级别 1 和级别 2。
级别 1 是最快的压缩，通常适用于短数据。
级别 2 稍微慢一些，但提供更好的压缩比。


请注意，压缩数据无论级别如何，都可以使用下面的函数 fastlz_decompress 进行解压缩。
*/

int fastlz_compress_level(int level, const void* input, int length, void* output);

/**
解压缩一个压缩数据块，并返回解压缩块的大小。如果发生错误，例如压缩数据损坏或输出缓冲区不够大，则返回 0（零）。


输入缓冲区和输出缓冲区不能重叠。


解压缩是内存安全的，并保证不会超过 maxout 指定的大小写入输出缓冲区。


请注意，解压缩始终有效，无论在上面的 fastlz_compress_level 中指定的压缩级别如何（在生成压缩块时）。
*/

int fastlz_decompress(const void* input, int length, void* output, int maxout);

#endif /* FASTLZ_H */