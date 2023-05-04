#include <windows.h>
#include <stdio.h>

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <shlwapi.h>

// compile with
// g++ main.cpp -o microPAM_convert.exe -lcomdlg32 -lshlwapi

void cVAS_Decode(uint32_t *out, uint32_t *inp, int nd, int nb, int NX)
{
    int kk=0;
    int nx=NX;
    for (int ii = 0; ii < nd; ii++)
    {
        {
            nx -= nb; 
            if(nx > 0)
            {
                out[ii] =  (inp[kk] >> nx);
            }
            else if(nx==0)
            {
                out[ii] =  inp[kk++];
                nx = NX; 
            }
            else if(nx<0)
            {
                out[ii] = (inp[kk++]   << (-nx)); 
                nx += NX; 
                out[ii] |= (inp[kk] >> nx) ;//& (1<<(32-nx)-1);
            }
        }
    }
}

int32_t *cVAS_DecodeData32(uint32_t *xx, int32_t *idat, int *nd, int *nb, int32_t ic1, int32_t *ndat)
{
    int32_t nch=xx[8];
    int32_t nsamp=128;
    int32_t NH=6;        // header size 

    int32_t  *tmp3=new int32_t[ic1*nch*nsamp];
    uint32_t *tmp4=new uint32_t[ic1*nch*nsamp];

    static int ic =-2;
    static uint32_t *tmp0=new uint32_t[nch];
    static uint32_t *tmp1=new uint32_t[nch*nsamp];

    uint32_t *tmp2=new uint32_t[nch*nsamp];

    for(int ii=0; ii<nch; ii++) tmp0[ii]=0;
    for(int ii=0; ii<ic1*nch*nsamp; ii++) tmp3[ii]=0;

    // check continuation
    int *nd1 = new int[ic1];
    int *nd2 = new int[ic1];

    // nd1 contains num of data if nd2==0 or of first bolock
    // nd2 contains num of data
    int kk=0;
    // check if first block is continuation
    for(int ii=0; ii<ic1; ii++)
    {
        nd1[ii]=nd[ii] & 0x0000ffff;
        nd2[ii]=nd[ii] >>16;
    }
    //
    int i0;
    if ((ic<0) & (nd2[0]>0) & (nd2[1]==0)) 
        i0=1;
    else
        i0=0;

    for(int ii=0; ii<ic1; ii++)
    {
        int j1=idat[ii]+NH; // start of tmp0
        int j2=j1+nch;      // start of tmp1
        int ndx=nd1[ii]-nch;    // number of data (if no continuation)

        if(nd2[ii]==0)
        {
            // normal block without continuation
            for(int jj=0; jj<nch; jj++)  tmp0[jj]=xx[j1+jj];
            for(int jj=0; jj<nch*nsamp; jj++) tmp1[jj]=0;
            for(int jj=0; jj<ndx; jj++) tmp1[jj]=xx[j2+jj];
            ic=0;
        }
        else
        {
            // block with continuation
            if(nd1[ii]==0)  // empty block
            {
                ic = -1;
            }
            else if(nd1[ii]==nd2[ii]) // following empty block
            {
                // normal block without continuation
                for(int jj=0; jj<nch; jj++)  tmp0[jj]=xx[j1+jj];
                for(int jj=0; jj<nch*nsamp; jj++) tmp1[jj]=0;
                for(int jj=0; jj<ndx; jj++) tmp1[jj]=xx[j2+jj];
                ic=0;
            }
            else
            {
                if(ic == -2) // should be first block
                    continue;    // have no continuation stored
                else if(ic==0)   // should be first of two continued blocks
                {
                    // store data
                    for(int jj=0; jj<nch; jj++) tmp0[jj]=xx[j1+jj];
                    for(int jj=0; jj<nch*nsamp; jj++) tmp1[jj]=0;
                    for(int jj=0; jj<ndx; jj++) tmp1[jj]=xx[j2+jj];
                    ic=1;
                }
                else
                {
                    // have continuation
                    // add actual data
                    for(int jj=0; jj<nd1[ii]; jj++) tmp1[nd2[ii]-nch-nd1[ii]+jj]=xx[j1+jj];
                    ic=0;
                }
            }
        }
        //
        if(ic==0)
        {
            int nbx=nb[ii]&0xffff;
            int nb1=1<<(nbx-1);
            int nb2=1<<nbx;
            int msk=nb2-1;

            cVAS_Decode(tmp2,tmp1,nch*nsamp,nbx,32);

            int32_t *tmp4i = (int32_t *)tmp4;
            for(int jj=0; jj<nch*nsamp; jj++)  
            {
                tmp4[jj] = tmp2[jj] & msk;
                if((tmp4[jj] & nb1) > 0) tmp4i[jj] -= nb2;
            }
            for(int jj=0; jj<nch; jj++) tmp4[jj]=tmp0[jj];

            for(int jj=nch; jj< nch*nsamp; jj++)  tmp4i[jj] += tmp4i[jj-nch];
            
            for(int jj=0; jj< nch*nsamp; jj++)  tmp3[kk*nch*nsamp+jj] = tmp4i[jj];
            kk += 1;
        }
    }

    *ndat = kk*nch*nsamp;

    delete[] tmp4;
    delete[] tmp2 ;
    
    return tmp3;
}

int32_t * cVAS_getData32(uint32_t *xx, int32_t n1, int32_t *ndat)
{
    int ic1=0;
    for(int ii=0; ii<n1;ii++) if(xx[ii]==0xA5A5A5A5) ic1++;

    int32_t *idat=new int32_t[ic1];
    int32_t *nd=new int32_t[ic1];
    int32_t *nb=new int32_t[ic1];

    int jj=0;
    for(int ii=0; ii<n1; ii++)
    { 
        if(xx[ii]==0xA5A5A5A5)
        {
            idat[jj]=ii;
            nb[jj]=xx[ii+1];
            nd[jj]=xx[ii+5];
            jj++;
        }
    }
    int32_t *tmp5=cVAS_DecodeData32(xx,idat,nd,nb,ic1, ndat);
    return tmp5;
}

long GetFileSize(char * filename)
{
    struct stat stat_buf;
    int rc = stat(filename, &stat_buf);
    return rc == 0 ? stat_buf.st_size : -1;
}

uint32_t *cVAS_LoadData(char * filename, int *n1)
{
    int32_t nbytes=GetFileSize(filename);
    if(nbytes<0) return 0;

    *n1=nbytes/4;
    uint32_t *xx=new uint32_t[*n1];

    FILE* fidinp = fopen(filename,"rb"); // important to use binary file
    fread(xx,4,*n1,fidinp);
    fclose(fidinp);
    return xx;
}

int32_t *cVAS_LoadAcoustics(char *filename,
                uint32_t *SerNum, char *recTimeStamp, 
                uint32_t *fs, int32_t *nch, int32_t *ndat)
{
    int32_t n1=0;
    uint32_t *xx=cVAS_LoadData(filename,&n1);
    if(xx==0)
    {
        printf("%s not found\n",filename);
        return 0;
    }

    uint32_t vers=xx[5];
    *SerNum=xx[6];
    *fs = xx[7];    // sampling frequency
    *nch = xx[8];   // number of channels
    uint32_t cp = xx[12]; // data compress mode
    uint32_t sh = xx[13]; // shift in bits
    strncpy(recTimeStamp,(char *)&xx[1],16); recTimeStamp[16]=0;
    if(0) printf("%s: %d %8x %d %d %d %d\n",recTimeStamp, vers,*SerNum,*fs,*nch,cp,sh);

    int32_t *data=0;
    if((vers<10) || (vers>=20))
    {
        if(cp==0)
        {
            data=new int32_t[n1-128];
            memcpy(data,&xx[128],4*(n1-128));
        }
        else
        {
            data = cVAS_getData32(xx, n1, ndat);
        }
        if(0) printf("%d %d\n\n",n1,*ndat);
    }
    else
    {
        printf("Version %d not implemented\n",vers);
    }
    delete[] xx;

    return data;
}


char * wavHeader(uint32_t fileSize, int32_t fsamp, int32_t nchan, int32_t nbits)
{
//  int fsamp=48000;
//  int fsamp = fs;
//  int nchan=nch;

//  int nbits=16;
  int nbytes=nbits/8;

  int nsamp=fileSize/(nbytes*nchan);
  //
  static char wheader[44];
  //
  strcpy(wheader,"RIFF");
  strcpy(wheader+8,"WAVE");
  strcpy(wheader+12,"fmt ");
  strcpy(wheader+36,"data");
  *(int32_t*)(wheader+16)= 16;// chunk_size
  *(int16_t*)(wheader+20)= 1; // PCM 
  *(int16_t*)(wheader+22)=nchan;// numChannels 
  *(int32_t*)(wheader+24)= fsamp; // sample rate 
  *(int32_t*)(wheader+28)= fsamp*nbytes; // byte rate
  *(int16_t*)(wheader+32)=nchan*nbytes; // block align
  *(int16_t*)(wheader+34)=nbits; // bits per sample 
  *(int32_t*)(wheader+40)=nsamp*nchan*nbytes; 
  *(int32_t*)(wheader+4)=36+nsamp*nchan*nbytes; 

   return wheader;
}

void writeWav(char * outFilePath, 
                        int32_t *data, uint32_t SerNum, char recTimeStamp[],
                        int32_t fs, int32_t nch, uint32_t ndat)
{
    char foutName[80];
    sprintf(foutName,"%08X_%s.wav",SerNum,recTimeStamp);

    char outFile[MAX_PATH];
    strcpy(outFile, outFilePath);
    strcat(outFile,"\\");
    strcat(outFile, foutName);

    printf("%s\n",outFile);

    char *hdr= wavHeader(ndat*4, fs, nch, 32);

    FILE* fidout = fopen(outFile,"wb"); // important to use binary file
    fwrite(hdr, 1, 44, fidout);
    fwrite(data,4,ndat,fidout);
    fclose(fidout);
}

int32_t * bin2wav(char *inp, char *out)
{
    char recTimeStamp[20];
    uint32_t SerNum;
    uint32_t fs;
    int32_t nch;
    int32_t ndat;
    
    int32_t *data=0;

    data=cVAS_LoadAcoustics(inp,&SerNum,recTimeStamp, &fs, &nch, &ndat);
    if(data)
        writeWav(out, data, SerNum, recTimeStamp, fs, nch, ndat) ;       
    return data;
}
/****************************************************************************/
int main(int argc, char *argv[])
{
    if(0)
    {
        printf("**********************\n");
        for(int ii=0; ii<argc;ii++)
        {
            printf("%d %d %s\n",ii,argc,(char *)argv[ii]);
        }
        printf("**********************\n");
    }

    /***  for input */
    char iFilestring[MAX_PATH] = "\0";
    OPENFILENAMEA ipf={0};
    ipf.lStructSize = sizeof(OPENFILENAMEA);
    ipf.lpstrFile = iFilestring;
    ipf.nMaxFile = MAX_PATH;
    ipf.lpstrTitle = (char *) "Open bin file or directory (type x and press open)";

    char inFilePath[MAX_PATH];

    /***  for output */
    char oFilestring[MAX_PATH] = "\0";
    OPENFILENAMEA opf={0};
    opf.lStructSize = sizeof(OPENFILENAMEA);
    opf.lpstrFile = oFilestring;
    opf.nMaxFile = MAX_PATH;
    opf.lpstrTitle = (char *) "Open wav directory (type x and press open)";

    char outFilePath[MAX_PATH];

    char srcDir[MAX_PATH];
    char inFile[MAX_PATH];

    int single=0;
    int32_t *data=0;

    if(argc==1)
    {
        if(GetOpenFileNameA(&ipf))
        {
            strncpy(inFilePath, ipf.lpstrFile,MAX_PATH);

            char *ptr=inFilePath+strlen(inFilePath)-4;
            if(strcmp(ptr,".bin"))
            {   // no bin file selected
                // use directory path
                PathRemoveFileSpecA(inFilePath);
                printf(" Bin Path = %s\n",inFilePath);
            }
            else
            {   // bin file selected
                printf(" Bin File = %s\n",inFilePath);
                single=1;
            }
        }
        else
        {
            printf("must open a real file or dummy (x)\n");
            return 0;
        }

        if(GetOpenFileNameA(&opf))
        {
            strncpy(outFilePath, opf.lpstrFile,MAX_PATH);
            PathRemoveFileSpecA(outFilePath);
            printf(" Wav Path = %s\n",outFilePath);
        }
        else
        {
            printf("must open a dummy (x)\n");
            return 0;
        }
    }
    else if((argc==2))
    {
        printf("Format: microPAM_convert\n");
        printf("or\n");
        printf("Format: microPAM_convert  bin-directory  wav-directory\n\n");
        printf("or\n");
        printf("Format: microPAM_convert  bin-file  wav-directory\n\n");
        return 0;
    }
    else
    {
        strcpy(inFilePath, argv[1]);
        strcpy(outFilePath, argv[2]);
        char *ptr=inFilePath+strlen(inFilePath)-4;
        if(!strcmp(ptr,".bin"))
            single=1;
    }

    /*----------------------------------------------------------------*/

    if(single)
    {
        data=bin2wav(inFilePath, outFilePath);
        return 0;
    }

    /*-----------------------------------------------------------------*/
    
    strcpy(srcDir, inFilePath);
    strcat(srcDir, "\\*.bin");

    WIN32_FIND_DATAA ffd;
    HANDLE hFind = FindFirstFileA((LPCSTR)srcDir, &ffd);
    if (INVALID_HANDLE_VALUE == hFind) 
    {
       printf("Error FindFirstFile %s\n",srcDir);
       return -1;
    } 
    
    // List all the files in the directory with some info about them.
    do
    {
      if (ffd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
      {  printf("  %s   <DIR>\n", ffd.cFileName);
      }
      else
      { LARGE_INTEGER filesize;
        filesize.LowPart = ffd.nFileSizeLow;
        filesize.HighPart = ffd.nFileSizeHigh;
        printf("  %s   %ld bytes\n", ffd.cFileName, filesize.QuadPart);

        strcpy(inFile, inFilePath);
        strcat(inFile, "\\");
        strcat(inFile,ffd.cFileName);

        data=bin2wav(inFile, outFilePath);
      }
    }
    while (FindNextFileA(hFind, &ffd) != 0);

    if(data) delete[] data ;
    return 0;
}
