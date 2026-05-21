#include <vector>
#include <cstdio>
#include <cassert>
#include <zlib.h>

static bool writePNG(const char* fn, int W, int H, const std::vector<unsigned char>& raw) {
    std::vector<unsigned char> filtered;
    filtered.reserve((W*3+1)*H);
    for(int y=0;y<H;y++){
        filtered.push_back(0);
        for(int x=0;x<W*3;x++)
            filtered.push_back(raw[y*W*3+x]);
    }
    uLongf compSize=compressBound((uLong)filtered.size());
    std::vector<unsigned char> comp(compSize);
    if(compress2(comp.data(),&compSize,filtered.data(),(uLong)filtered.size(),6)!=Z_OK)return false;
    comp.resize(compSize);
    
    FILE* f=fopen(fn,"wb");if(!f)return false;
    auto writeU32=[&](unsigned v){unsigned char b[4]={(unsigned char)(v>>24),(unsigned char)(v>>16),(unsigned char)(v>>8),(unsigned char)v};fwrite(b,1,4,f);};
    auto writeChunk=[&](const char* tag,const unsigned char* data,size_t len){
        writeU32((unsigned)len);
        fwrite(tag,1,4,f);
        if(len>0)fwrite(data,1,len,f);
        unsigned crc=crc32(crc32(0L,Z_NULL,0),(const Bytef*)tag,4);
        if(len>0)crc=crc32(crc,data,(uInt)len);
        writeU32(crc);
    };
    unsigned char sig[8]={137,80,78,71,13,10,26,10};fwrite(sig,1,8,f);
    unsigned char ihdr[13]={(unsigned char)(W>>24),(unsigned char)(W>>16),(unsigned char)(W>>8),(unsigned char)W,
                             (unsigned char)(H>>24),(unsigned char)(H>>16),(unsigned char)(H>>8),(unsigned char)H,
                             8,2,0,0,0};
    writeChunk("IHDR",ihdr,13);
    writeChunk("IDAT",comp.data(),comp.size());
    unsigned char iend[1]={0};
    writeU32(0);fwrite("IEND",1,4,f);
    unsigned crc=crc32(crc32(0L,Z_NULL,0),(const Bytef*)"IEND",4);
    writeU32(crc);
    fclose(f);return true;
}

int main(){
    int W=100,H=100;
    std::vector<unsigned char> raw(W*H*3);
    for(int y=0;y<H;y++)for(int x=0;x<W;x++){
        int i=(y*W+x)*3;
        raw[i+0]=255; raw[i+1]=0; raw[i+2]=0; // red
        if(x>50){raw[i+0]=0;raw[i+1]=0;raw[i+2]=255;} // blue right half
        if(y>50){raw[i+0]=0;raw[i+1]=255;raw[i+2]=0;} // green bottom half
    }
    writePNG("test_out.png",W,H,raw);
    printf("Written test_out.png\n");
    return 0;
}
