#include "compress_decompress.h"

#include <stdint.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"


#define FASTLZ_LIKELY(c) (__builtin_expect(!!(c), 1))
#define FASTLZ_UNLIKELY(c) (__builtin_expect(!!(c), 0))

#define FLZ_ARCH64

#include <string.h>

// fastlz_memmove: 安全地将count字节从src移动到dest，支持重叠区域。
// 1.判断是否可以直接用标准库的 memmove：
// 如果 count > 4 并且 dest 在 src + count 之后（即目标和源没有重叠，或者目标在源后面），直接调用 memmove，效率高。
// 2.否则，自己实现拷贝：
// 用 switch-case 处理 count 为 0~3 的情况，避免循环开销。
// 对于 count > 3 的情况，使用 do-while 循环逐字节拷贝。
// 3.switch-case 的技巧：
// 没有 break，利用 case 穿透（fallthrough），比如 count=3 时，会连续执行 3、2、1 的拷贝。
static void fastlz_memmove(uint8_t* dest, const uint8_t* src, uint32_t count) {
  if ((count > 4) && (dest >= src + count)) {
    memmove(dest, src, count);
  } else {
    switch (count) {
      default:
        do {
          *dest++ = *src++;
        } while (--count);
        break;
      case 3:
        *dest++ = *src++;
      case 2:
        *dest++ = *src++;
      case 1:
        *dest++ = *src++;
      case 0:
        break;
    }
  }
}

// fastlz_memcpy: 简单地将count字节从src复制到dest，不处理重叠。
static void fastlz_memcpy(uint8_t* dest, const uint8_t* src, uint32_t count) { memcpy(dest, src, count); }

// flz_readu32: 从ptr读取一个32位无符号整数（小端）。
// 用32位是为了兼容性、效率和算法本身的需求。即使在64位平台，32位操作依然高效且足够。
static uint32_t flz_readu32(const void* ptr) { return *(const uint32_t*)ptr; }

// flz_cmp: 比较p和q指向的数据，直到r，返回相同字节数。
// p：指向当前待比较的输入数据的当前位置（通常是当前压缩窗口的末尾）。
// q：指向历史数据中可能匹配的位置（通常是哈希表查到的历史片段）。
// r：指向输入数据的比较结束位置（通常是输入数据的末尾或窗口的末尾）。
static uint32_t flz_cmp(const uint8_t* p, const uint8_t* q, const uint8_t* r) {
  const uint8_t* start = p;

  if (flz_readu32(p) == flz_readu32(q)) {
    p += 4;
    q += 4;
  }
  while (q < r)
    if (*p++ != *q++) break;
  return p - start;
}

// 含义：一次最多原样拷贝（literal）的字节数。
// 原因：32字节是一个合适的批量，既能减少控制字节数量，又不会让未压缩数据块太大，便于实现和效率优化。
#define MAX_COPY 32
// 含义：一次匹配（重复数据）的最大长度。
// 原因：FastLZ 的匹配长度编码方式决定了最大长度为 264 字节（256+8），这样可以用较少的字节编码较长的匹配。
#define MAX_LEN 264 /* 256 + 8 */
// 含义：匹配距离的最大值（即历史窗口的最大回溯距离）。
// 原因：8192（8K）是LZ类压缩算法常用的窗口大小，既能找到足够多的重复，又不会占用太多内存。L1和L2是不同压缩级别的窗口设置。
#define MAX_L1_DISTANCE 8192
#define MAX_L2_DISTANCE 8191
// 含义：极限情况下的最大回溯距离。
// 原因：用于支持更远距离的匹配，便于编码更长距离的重复数据。
#define MAX_FARDISTANCE (65535 + MAX_L2_DISTANCE - 1)
// 含义：哈希表的大小和掩码。
// 原因：2^13=8192，哈希表大小与窗口大小一致，便于快速查找历史数据。HASH_MASK 用于快速取模，保证哈希值在表内。
#define HASH_LOG 13
#define HASH_SIZE (1 << HASH_LOG)
#define HASH_MASK (HASH_SIZE - 1)

// flz_hash: 计算32位值的哈希，用于查找匹配。
static uint16_t flz_hash(uint32_t v) {
// 2654435769 是著名的 Knuth 哈希常数（0x9E3779B9），用于将输入值打散，减少哈希冲突。
// 右移19位，就是取原始32位整数的高13位，作为哈希表下标。
  uint32_t h = (v * 2654435769LL) >> (32 - HASH_LOG);
  return h & HASH_MASK;
}

// flz_smallcopy: 拷贝最多MAX_COPY字节，优化小块拷贝。
// 1.优化大于等于4字节的拷贝：
// 如果 count >= 4，就把源和目标都按4字节（uint32_t）为单位进行拷贝，这样比逐字节拷贝更快。
// 每次拷贝4字节，循环直到剩下不超过4字节。
// 2.处理剩余的1~3字节：
// 拷贝完4字节对齐的部分后，剩下的1~3字节用 fastlz_memcpy 逐字节拷贝，保证所有数据都被复制。
static void flz_smallcopy(uint8_t* dest, const uint8_t* src, uint32_t count) {
  if (count >= 4) {
    const uint32_t* p = (const uint32_t*)src;
    uint32_t* q = (uint32_t*)dest;
    while (count > 4) {
      *q++ = *p++;
      count -= 4;
      dest += 4;
      src += 4;
    }
  }
  fastlz_memcpy(dest, src, count);
}

// flz_maxcopy: 拷贝正好MAX_COPY字节，优化为8次32位拷贝。
// 这个函数假定要拷贝的数据长度正好是 32 字节（8 × 4 字节）。
// 通过 8 次 4 字节（uint32_t）赋值，快速完成拷贝，比逐字节拷贝更高效。
static void flz_maxcopy(void* dest, const void* src) {
  const uint32_t* p = (const uint32_t*)src;
  uint32_t* q = (uint32_t*)dest;
  *q++ = *p++;
  *q++ = *p++;
  *q++ = *p++;
  *q++ = *p++;
  *q++ = *p++;
  *q++ = *p++;
  *q++ = *p++;
  *q++ = *p++;
}

// flz_literals: 将runs个字节的原始数据从src复制到dest，处理分段。
// 1.分段处理大于等于 MAX_COPY 的数据块：
// 每次最多处理 MAX_COPY（32）字节。
// 先写一个控制字节 MAX_COPY-1（表示后面跟着 32 字节原始数据）。
// 用 flz_maxcopy 拷贝 32 字节。
// 更新指针和剩余字节数，循环直到剩下不足 32 字节。
// 2.处理剩余不足 MAX_COPY 的数据：
// 如果还有剩余（1~31字节），写一个控制字节 runs-1（表示后面跟着 runs 字节原始数据）。
// 用 flz_smallcopy 拷贝剩余字节。
// 3.返回新的 dest 指针：
// 返回写入数据后的 dest 位置，方便后续继续写数据。
static uint8_t* flz_literals(uint32_t runs, const uint8_t* src, uint8_t* dest) {
  while (runs >= MAX_COPY) {
    *dest++ = MAX_COPY - 1;// 控制字节 31 (00011111)
    flz_maxcopy(dest, src);
    src += MAX_COPY;
    dest += MAX_COPY;
    runs -= MAX_COPY;
  }
  if (runs > 0) {
    *dest++ = runs - 1;// 控制字节 0~30 (000xxxxx)
    flz_smallcopy(dest, src, runs);
    dest += runs;
  }
  return dest;
}

// flz1_match: 写入fastlz1压缩格式的匹配信息到op，返回新op。
// 1.distance 预处理：
// --distance;
// fastlz1 的距离是以 1 为基准的，所以编码时要减 1。
// 2.处理超长匹配：
// 如果 len > MAX_LEN - 2（即长度太长，单个 match 段放不下），就拆成多个最大长度的段，每段长度为 MAX_LEN - 2。
// 每段编码为 3 字节：
//   第1字节：(7 << 5) + (distance >> 8)，高3位为7，低5位为距离高位
//   第2字节：MAX_LEN - 2 - 7 - 2，即253
//   第3字节：distance & 255，距离低8位
// 3.处理剩余长度：
//   如果剩余长度小于7，编码为2字节：
//     第1字节：(len << 5) + (distance >> 8)
//     第2字节：distance & 255
//   如果剩余长度大于等于7，编码为3字节：
//     第1字节：(7 << 5) + (distance >> 8)
//     第2字节：len - 7
//     第3字节：distance & 255
// 4.返回输出指针：
// 返回写入数据后的新输出指针。
// -------------------------------------------------------------------
// MAX_L1_DISTANCE=8192=2^13，distance >> 8取高5位，distance & 255取低8位。
// 0 <= len <= 6，用2字节编码：        [len(3bit) | distance高5bit]    ||    [distance低8bit]
//                                最大len为6，最大distance为8191
// 7 <= len <= 262，用3字节编码：     [111 | distance高5bit]           ||    [扩展len(0~253)]    ||    [distance低8bit]
//                                len>=7时：第一字节高3bit为111表示扩展；第二字节为扩展长度，最大253
//                                实际len=7+扩展len，最大为260（即扩展len=253时）
// len >= 263，每262字节为一段（3字节编码），多段连接。最后剩余（1~262）字节再用单段处理。
//                                每段都是[111 | distance高5bit] || [253] || [distance低8bit]
//                                注意：每段循环len -= 262，但每三字节块最大只能编码260字节（7+253）；剩下的1~2字节由后续小块补齐
//                                eg：len=263进入while循环，剩下1字节，会再用2字节编码补齐
// -------------------------------------------------------------------
// 比如263进入循环的话，op只能记录7+253=260，len-262=1，还剩1字节，跳出while循环，再用2字节编码完
// 比如262不进入循环的话，op直接记录7+255=262，三个字节直接可以编码完
static uint8_t* flz1_match(uint32_t len, uint32_t distance, uint8_t* op) {
  --distance;
  if (FASTLZ_UNLIKELY(len > MAX_LEN - 2))
    // 每轮减262，却只写253（表示253+7=260字节）
    // 这是实现上的一种特殊处理，让算法结构简化，剩下的字节正是交给后面的if/else语句用两字节编码来覆盖
    while (len > MAX_LEN - 2) {
      *op++ = (7 << 5) + (distance >> 8);// op的高3位(第一字节的高3bit)变为7
      *op++ = MAX_LEN - 2 - 7 - 2;// op的低8位(第二字节的8bit)变为253
      *op++ = (distance & 255);
      len -= MAX_LEN - 2;// len -= 262;
    }
  if (len < 7) {
    *op++ = (len << 5) + (distance >> 8);
    *op++ = (distance & 255);
  } else {
    *op++ = (7 << 5) + (distance >> 8);
    *op++ = len - 7;
    *op++ = (distance & 255);
  }
  return op;
}
// len：匹配到的额外字节数；uint32_t len = flz_cmp(ref + 3, ip + 3, ip_bound); 返回的是从第4字节开始继续匹配的字节数。
// distance：匹配距离（当前数据与历史数据的距离）。
// op：输出缓冲区指针。

#define FASTLZ_BOUND_CHECK(cond) \
  if (FASTLZ_UNLIKELY(!(cond))) return 0;

// fastlz1_compress: fastlz1压缩主函数，将input压缩到output。
static int fastlz1_compress(const void* input, int length, void* output) {
  const uint8_t* ip = (const uint8_t*)input;
  const uint8_t* ip_start = ip;
  const uint8_t* ip_bound = ip + length - 4; /* because readU32 */
  const uint8_t* ip_limit = ip + length - 12 - 1;
  uint8_t* op = (uint8_t*)output;

  uint32_t htab[HASH_SIZE];
  uint32_t seq, hash;

  for (hash = 0; hash < HASH_SIZE; ++hash) htab[hash] = 0;

  // 1.主循环第一次遇到匹配时，会把 anchor 到 ip 之间的数据（即前2字节）作为 literal 块输出！！！对应下面注释
  const uint8_t* anchor = ip;
  ip += 2;

  while (FASTLZ_LIKELY(ip < ip_limit)) {
    const uint8_t* ref;
    uint32_t distance, cmp;

    do {
      seq = flz_readu32(ip) & 0xffffff;
      hash = flz_hash(seq);
      ref = ip_start + htab[hash];
      htab[hash] = ip - ip_start;
      distance = ip - ref;
      cmp = FASTLZ_LIKELY(distance < MAX_L1_DISTANCE) ? flz_readu32(ref) & 0xffffff : 0x1000000;
      if (FASTLZ_UNLIKELY(ip >= ip_limit)) break;
      ++ip;
    } while (seq != cmp);

    if (FASTLZ_UNLIKELY(ip >= ip_limit)) break;
    // do-while找到匹配时，记录htab，最后++ip;
    // 跳出循环匹时，此时的 ip 已经指向“第一个配字节的下一个位置”，需要回退一步（--ip）。
    --ip;

    // 2.主循环第一次遇到匹配时，会把 anchor 到 ip 之间的数据（即前2字节）作为 literal 块输出！！！对应上面注释
    if (FASTLZ_LIKELY(ip > anchor)) {
      op = flz_literals(ip - anchor, anchor, op);
    }
    
    //    ref      ip
    // eg：a  b  c  a  b  c  a  b  c  d ...
    //     0  1  2  3  4  5  6  7  8  9 ...
    // 第一次do-while退出时，ref=0 ip=3，len=3
    // 然后继续从 012的abc 和 345的abc 的后面比较，直到不匹配或到达边界
    // len = flz_cmp(ref + 3, ip + 3, ip_bound) 即 3 = flz_cmp(3, 6, ip_bound)
    uint32_t len = flz_cmp(ref + 3, ip + 3, ip_bound);
    // 把len和distance以压缩格式写进op
    op = flz1_match(len, distance, op);

    /* update the hash at match boundary */
    // 最后补2次hash，每次都是补match末尾的2个滑窗[三字节]
    // 如果一个都不补，会错失后续依赖match块内3字节串得分支；如果全补，慢；只补2次，够用也快
    // eg1：abcabcabcd，最后一次match后，补的2次hash分别是（a b c）和（b c d），其中ip += len 即 ip = ip + len = 3 + 3 = 6
    // eg2：abcxyzabcxmn，最后一次match后，补的2次hash分别是（b c x）和（c x m），其中ip += len 即 ip = ip + len = 6 + 1 = 7
    // eg3：a b c A B C D E F a b c A B C x y z，最后补[12,13,14] = A B C 和 [13,14,15] = B C x
    // eg4：a b c A B C D E F g a b c A B C D x y z，最后补[14,15,16]: B C D 和 [15,16,17]: C D x
    // eg5：a  b  c  A  B  C  D  E  F  G  H  I  J  K   a  b  c  A  B  C  D  E  F  G  x  y  z，最后补[21,22,23] = E F G 和 [22,23,24] = F G x
    ip += len; // ip = ip + len = 3 + 3 = 6
    seq = flz_readu32(ip); // seq = 6789的abcd
    hash = flz_hash(seq & 0xffffff); // seq & 0xffffff = 678的abc(小端)
    htab[hash] = ip++ - ip_start; // htab[hash(a b c)] = 6，ip=7
    seq >>= 8; // seq = 789的bcd
    hash = flz_hash(seq);
    htab[hash] = ip++ - ip_start; // htab[hash(b c d)] = 7，ip=8

    anchor = ip; // 8
  }

  // 如上eg1：abcabcabcd，copy = 0 + 10 - 8 = 2，然后复制最后89的cd两个变量到op
  // (uint8_t*)input + length：这是输入数据的末尾地址（指针）。
  // anchor：这是最后一次匹配后，未被压缩的原始数据块的起始地址（指针）。
  // 两个指针相减，结果是字节数，即“从anchor到input末尾有多少字节”。
  uint32_t copy = (uint8_t*)input + length - anchor;
  op = flz_literals(copy, anchor, op);
  // 返回压缩后数据的长度
  return op - (uint8_t*)output;
}
// const uint8_t* ip：输入数据当前处理位置的指针（input pointer）。
// const uint8_t* ip_start：输入数据的起始地址，用于计算偏移量。
// const uint8_t* ip_bound：输入数据的边界指针，等于 ip + length - 4，用于保证后续 flz_readu32 不越界。
// const uint8_t* ip_limit：输入数据的主循环终止指针，等于 ip + length - 12 - 1，保证压缩主循环不会越界。
// uint8_t* op：输出数据当前写入位置的指针（output pointer）。
// uint32_t htab[HASH_SIZE]：哈希表，保存每个哈希值最近一次出现的位置（偏移量），用于快速查找历史数据。
// uint32_t seq：当前处理位置的3字节序列（24位），用于哈希和匹配。
// uint32_t hash：当前序列 seq 的哈希值，用于查找哈希表。
// const uint8_t* anchor：当前未匹配（literal）数据块的起始位置指针。
// const uint8_t* ref：哈希表查到的历史数据位置指针，用于匹配。
// uint32_t distance：当前输入位置与历史匹配位置 ref 的距离（即回溯距离）。
// uint32_t cmp：历史数据与当前数据的3字节序列比较结果，用于判断是否匹配。
// uint32_t len：匹配到的最长重复字节数。
// uint32_t copy：循环结束后，剩余未匹配数据的字节数。

// fastlz1_decompress: fastlz1解压主函数，将input解压到output。
// ---后期删除-----重复注释--------------------重复注释----------------重复注释---------后期删除--------------
// MAX_L1_DISTANCE=8192=2^13，distance >> 8取高5位，distance & 255取低8位。
// 0 <= len <= 6，用2字节编码：        [len(3bit) | distance高5bit]    ||    [distance低8bit]
//                                最大len为6，最大distance为8191
// 7 <= len <= 262，用3字节编码：     [111 | distance高5bit]           ||    [扩展len(0~253)]    ||    [distance低8bit]
//                                len>=7时：第一字节高3bit为111表示扩展；第二字节为扩展长度，最大253
//                                实际len=7+扩展len，最大为260（即扩展len=253时）
// len >= 263，每262字节为一段（3字节编码），多段连接。最后剩余（1~262）字节再用单段处理。
//                                每段都是[111 | distance高5bit] || [253] || [distance低8bit]
//                                注意：每段循环len -= 262，但每三字节块最大只能编码260字节（7+253）；剩下的1~2字节由后续小块补齐
//                                eg：len=263进入while循环，剩下1字节，会再用2字节编码补齐
// ---后期删除------重复注释---------------重复注释--------------------重复注释-------后期删除----------------
// literal 块：控制字节高3位全0（000xxxxx），长度 = xxxxx + 1
// match 块：控制字节高3位非0（yyyxxxxx），长度和距离编码在 yyy 和 xxxxx 及后续字节
static int fastlz1_decompress(const void* input, int length, void* output, int maxout) {
  const uint8_t* ip = (const uint8_t*)input;
  const uint8_t* ip_limit = ip + length;
  const uint8_t* ip_bound = ip_limit - 2;
  uint8_t* op = (uint8_t*)output;
  uint8_t* op_limit = op + maxout;
  // 取 input 的第一个字节的低 5 位作为控制字节
  uint32_t ctrl = (*ip++) & 31;
  // 主循环第一次遇到匹配时，会把 anchor 到 ip 之间的数据（即前2字节）作为 literal 块输出（fastlz1_compress注释）
  // 只有第一次循环时，ctrl = (*ip++) & 31;，此时 ctrl 只取了低5位（0~31）。
  // 后续每次循环，ctrl = *ip++;，直接取一个字节（0~255），没有再与31做与运算，所以 ctrl 可能大于等于32。
  while (1) {
    // ctrl >= 32   =====>   高3位非0   =====>   是 match 段   =====>   1<=len(也就是ctrl的高3位)<=7
    if (ctrl >= 32) {// #define MAX_COPY 32
      uint32_t len = (ctrl >> 5) - 1;//  -1 是为了让高3位值1、2、3...正好对应最短3、4、5...字节的match（x-1+3即x+2，下面有len += 3;）
      if (len == 7 - 1) {//如果长度等于6（7-1），说明是3字节编码的情况，继续读一个字节加到 len 上
        FASTLZ_BOUND_CHECK(ip <= ip_bound);
        len += *ip++;
      }
      len += 3;// 匹配长度加3

      uint32_t ofs = (ctrl & 31) << 8;// 低5位，左移8，得高位部分（ofs = DDDDD00000000）
      // offset=0 的含义就是“回到输出缓冲区的前一个已经解压好的字节”
      // flz1_match中有注释：fastlz1 的距离是以 1 为基准的，所以编码时要减 1
      const uint8_t* ref = op - ofs - 1; // 1.计算历史数据的起始位置第一步：先减去高5位的偏移量（ofs），再减1是因为fastlz1的匹配是从ref的第一个字节开始的，而op指向的是下一个要写入的位置！！！对应下面注释
      ref -= *ip++;// 2.计算历史数据的起始位置第二步：再减去低8位！！！对应上面注释
      
      FASTLZ_BOUND_CHECK(op + len <= op_limit);
      FASTLZ_BOUND_CHECK(ref >= (uint8_t*)output);
      fastlz_memmove(op, ref, len);// 从 ref 拷贝 len 字节到 op
      op += len;// op 前进 len
    // ctrl < 32   =====>   高3位为0   =====>   是 literal 段   =====>   len(也就是ctrl的高3位)=0
    } else {
      ctrl++;// 表示 literal 长度（1~32字节），flz_literals存储控制字节会-1，即1~32长度->0~31的5bit，需要+1恢复
      FASTLZ_BOUND_CHECK(op + ctrl <= op_limit);
      FASTLZ_BOUND_CHECK(ip + ctrl <= ip_limit);
      fastlz_memcpy(op, ip, ctrl);// 从 input 拷贝 ctrl 字节到 output
      ip += ctrl;// ip 前进 ctrl
      op += ctrl;// op 前进 ctrl
    }

    if (FASTLZ_UNLIKELY(ip > ip_bound)) break;
    // 读取下一个控制字节
    ctrl = *ip++;
  }
  // 返回解压后数据长度
  return op - (uint8_t*)output;
}
// ip：输入数据当前读取指针
// ip_limit：输入数据末尾指针
// ip_bound：输入边界（防止越界）
// op：输出数据当前写入指针
// op_limit：输出数据末尾指针
// ctrl：控制字节，决定接下来是 literal 还是 match

// flz2_match: 写入fastlz2压缩格式的匹配信息到op，返回新op。
static uint8_t* flz2_match(uint32_t len, uint32_t distance, uint8_t* op) {
  --distance;
  if (distance < MAX_L2_DISTANCE) {
    if (len < 7) {
      *op++ = (len << 5) + (distance >> 8);
      *op++ = (distance & 255);
    } else {
      *op++ = (7 << 5) + (distance >> 8);
      for (len -= 7; len >= 255; len -= 255) *op++ = 255;
      *op++ = len;
      *op++ = (distance & 255);
    }
  } else {
    /* far away, but not yet in the another galaxy... */
    if (len < 7) {
      distance -= MAX_L2_DISTANCE;
      *op++ = (len << 5) + 31;
      *op++ = 255;
      *op++ = distance >> 8;
      *op++ = distance & 255;
    } else {
      distance -= MAX_L2_DISTANCE;
      *op++ = (7 << 5) + 31;
      for (len -= 7; len >= 255; len -= 255) *op++ = 255;
      *op++ = len;
      *op++ = 255;
      *op++ = distance >> 8;
      *op++ = distance & 255;
    }
  }
  return op;
}

// fastlz2_compress: fastlz2压缩主函数，将input压缩到output。
static int fastlz2_compress(const void* input, int length, void* output) {
  const uint8_t* ip = (const uint8_t*)input;
  const uint8_t* ip_start = ip;
  const uint8_t* ip_bound = ip + length - 4; /* because readU32 */
  const uint8_t* ip_limit = ip + length - 12 - 1;
  uint8_t* op = (uint8_t*)output;

  uint32_t htab[HASH_SIZE];
  uint32_t seq, hash;

  /* initializes hash table */
  for (hash = 0; hash < HASH_SIZE; ++hash) htab[hash] = 0;

  /* we start with literal copy */
  const uint8_t* anchor = ip;
  ip += 2;

  /* main loop */
  while (FASTLZ_LIKELY(ip < ip_limit)) {
    const uint8_t* ref;
    uint32_t distance, cmp;

    /* find potential match */
    do {
      seq = flz_readu32(ip) & 0xffffff;
      hash = flz_hash(seq);
      ref = ip_start + htab[hash];
      htab[hash] = ip - ip_start;
      distance = ip - ref;
      cmp = FASTLZ_LIKELY(distance < MAX_FARDISTANCE) ? flz_readu32(ref) & 0xffffff : 0x1000000;
      if (FASTLZ_UNLIKELY(ip >= ip_limit)) break;
      ++ip;
    } while (seq != cmp);

    if (FASTLZ_UNLIKELY(ip >= ip_limit)) break;

    --ip;

    /* far, needs at least 5-byte match */
    if (distance >= MAX_L2_DISTANCE) {
      if (ref[3] != ip[3] || ref[4] != ip[4]) {
        ++ip;
        continue;
      }
    }

    if (FASTLZ_LIKELY(ip > anchor)) {
      op = flz_literals(ip - anchor, anchor, op);
    }

    uint32_t len = flz_cmp(ref + 3, ip + 3, ip_bound);
    op = flz2_match(len, distance, op);

    /* update the hash at match boundary */
    ip += len;
    seq = flz_readu32(ip);
    hash = flz_hash(seq & 0xffffff);
    htab[hash] = ip++ - ip_start;
    seq >>= 8;
    hash = flz_hash(seq);
    htab[hash] = ip++ - ip_start;

    anchor = ip;
  }

  uint32_t copy = (uint8_t*)input + length - anchor;
  op = flz_literals(copy, anchor, op);

  // fastlz2 压缩时，首字节高3位写为 001
  *(uint8_t*)output |= (1 << 5);

  return op - (uint8_t*)output;
}

// fastlz2_decompress: fastlz2解压主函数，将input解压到output。
static int fastlz2_decompress(const void* input, int length, void* output, int maxout) {
  const uint8_t* ip = (const uint8_t*)input;
  const uint8_t* ip_limit = ip + length;
  const uint8_t* ip_bound = ip_limit - 2;
  uint8_t* op = (uint8_t*)output;
  uint8_t* op_limit = op + maxout;
  uint32_t ctrl = (*ip++) & 31;

  while (1) {
    if (ctrl >= 32) {
      uint32_t len = (ctrl >> 5) - 1;
      uint32_t ofs = (ctrl & 31) << 8;
      const uint8_t* ref = op - ofs - 1;

      uint8_t code;
      if (len == 7 - 1) do {
          FASTLZ_BOUND_CHECK(ip <= ip_bound);
          code = *ip++;
          len += code;
        } while (code == 255);
      code = *ip++;
      ref -= code;
      len += 3;

      /* match from 16-bit distance */
      if (FASTLZ_UNLIKELY(code == 255))
        if (FASTLZ_LIKELY(ofs == (31 << 8))) {
          FASTLZ_BOUND_CHECK(ip < ip_bound);
          ofs = (*ip++) << 8;
          ofs += *ip++;
          ref = op - ofs - MAX_L2_DISTANCE - 1;
        }

      FASTLZ_BOUND_CHECK(op + len <= op_limit);
      FASTLZ_BOUND_CHECK(ref >= (uint8_t*)output);
      fastlz_memmove(op, ref, len);
      op += len;
    } else {
      ctrl++;
      FASTLZ_BOUND_CHECK(op + ctrl <= op_limit);
      FASTLZ_BOUND_CHECK(ip + ctrl <= ip_limit);
      fastlz_memcpy(op, ip, ctrl);
      ip += ctrl;
      op += ctrl;
    }

    if (FASTLZ_UNLIKELY(ip >= ip_limit)) break;
    ctrl = *ip++;
  }

  return op - (uint8_t*)output;
}

// fastlz_decompress: 自动检测压缩级别并解压，支持level 1和2。
int fastlz_decompress(const void* input, int length, void* output, int maxout) {
  /* magic identifier for compression level */
  int level = ((*(const uint8_t*)input) >> 5) + 1;

  if (level == 1) return fastlz1_decompress(input, length, output, maxout);
  if (level == 2) return fastlz2_decompress(input, length, output, maxout);

  /* unknown level, trigger error */
  return 0;
}

// fastlz_compress_level: 根据level选择压缩算法，1为fastlz1，2为fastlz2。
int fastlz_compress_level(int level, const void* input, int length, void* output) {
  if (level == 1) return fastlz1_compress(input, length, output);
  if (level == 2) return fastlz2_compress(input, length, output);

  return 0;
}

#pragma GCC diagnostic pop
