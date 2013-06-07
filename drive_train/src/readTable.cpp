#include <memory.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <string>

int readTable(char *fileName, int *numSteps, int *numCols, double **goalTab)
{
    //open the file
    FILE *fd;
    fd = fopen(fileName,"r");
    char commentStr[2048];
    
    // Exit if file opening failed
    if (fd == NULL){
        return(-1);
    }

    //read in the first two lines and throw them away:33
    fgets(commentStr,2048,fd);
    fgets(commentStr,2048,fd);

    //read in the size
    fscanf(fd,"%d",numSteps);
    fscanf(fd,"%d",numCols);

    //create the matrix
    *goalTab = (double *) malloc(sizeof(double)*(*numSteps)*(*numCols));
    double *ptr;
    ptr = *goalTab;

    //read in the table
    for(int loopX = 0; loopX < *numSteps; loopX++)
    {
        //remove the beginning comment
        fscanf(fd,"%s",commentStr);
      for(int loopY = 0; loopY < *numCols; loopY++)
      {
         fscanf(fd,"%lf",ptr);
         ptr++;
      }
    }

    fclose(fd);
 
    return(0);
  
}

/*
int main(int argc, char **argv)
{

    int n;
    int m;
    double *data;
    int loop = 0;
    //char *fileName;
    std::string fileName("goalMap.txt");

    int val = readTable((char *) fileName.c_str(),&n,&m,&data);

    double *ptr;
    ptr = data;

    printf("%d\n",n); 
    printf("%d\n",m);
    for(int loopX = 0; loopX < n; loopX++)
    {
      for(int loopY = 0; loopY < m; loopY++)
      {
        printf("%2.2lf ",ptr[loopY]);
      }
      printf("\n");
        ptr +=  m;
    }

   free(data);

}


*/ 
