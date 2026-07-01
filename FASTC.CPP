/*
  MIT License

  Copyright (c) 2026 Stefan Mader

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/
/*
  This is a novel algorithm by me (Stefan Mader) to find centres of 3D objects in
  2d point clouds. Please don't say you have invented it. Atleast an AI found no
  other matching algorithm to this problem. It can detect the centre of (a) 3D 
  object(s) that is/are rotated in a scene.  I call it "Fast center". Some more work
  maybe needed on it to make it fully usable.
*/
#include <dos.h>
#include <conio.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#define POINTCOUNT 0x4000
#define CLUSTERSIZE 0x100

double pointX[POINTCOUNT];
double pointY[POINTCOUNT];

unsigned char screen[640/8*480*4];

void setBiosGraphicsMode(unsigned short mode) {
  union REGS regs;
  regs.w.ax = mode;
  int386(0x10, &regs, &regs);
}
 
unsigned short getBiosGraphicsMode() {
  union REGS regs;
  regs.w.ax = 0x0f00;
  int386(0x10, &regs, &regs);
  return regs.h.al;
}

void clearScreenRam() {
  int y,j,x;
  for (y = 0; y < 480; y++) {
    for (j = 0; j < 4; j++) { // j = bit plane
      outp(0x3c4, 2);
      outp(0x3c5, 1<<j);
      for (x = 0; x < 640 / 8 / 4; x++) {
        *(unsigned int *)(0xa0000+y*(640/8)+x*4) = 0;
      }
    }
  }
  memset(screen,0,sizeof(screen));
}

void setPixel(int x, int y, int col) {
  if ((unsigned int)x < 640 && (unsigned int)y < 480) {
    for (int j = 0; j < 4; j++) {
      outp(0x3c4, 2);
      outp(0x3c5, 1<<j);
      int b = col & (1<<j) ? 1 : 0;
      if (b) screen[y*(640/8)+x/8+j*640/8*480] |= 1<<(7-(x & 7));
      if (!b) screen[y*(640/8)+x/8+j*640/8*480] &= 255-(1<<(7-(x & 7)));
      *(unsigned char *)(0xa0000+y*(640/8)+x/8) = screen[y*(640/8)+x/8+j*640/8*480];
    }
  }
}

double kx[2],ky[2];

void initPoints() {
  if (false) {
    for (int i = 0; i < POINTCOUNT; i+=CLUSTERSIZE) {
      int cx = rand() % 640;
      int cy = rand() % 480;
      cx *= 0.5;
      cy *= 0.5;
      for (int j = 0; j < CLUSTERSIZE; j++) {
        pointX[i+j] = cx + (rand() % 50)-25;
        pointY[i+j] = cy + (rand() % 50)-25;
      }
    }
  } else {
    // anscheinend bekommt man so den mittelpunkt einer 3dimensionalen punktwolke (alles in 2d)
    double xadd = 50*1;
    double yadd = 100*0;
    double angle = 40*3.14159/180.0;
    for (int i = 0; i < POINTCOUNT; i++) {
      if (i >= POINTCOUNT/2) angle = -angle;
      double x = (rand() % 101)-50;
      double y = (rand() % 101)-50;
      double z = 100 + (rand() % 1001);
/*      int k = (i / 8)*1+10;
      if ((i & 1)==0) x = -k;
      if ((i & 1)!=0) x = +k;
      if ((i & 2)==0) y = -k;
      if ((i & 2)!=0) y = +k;
      if ((i & 4)==0) z = k*2+100;
      if ((i & 4)!=0) z = 100;*/
      if (i == 0 || i == POINTCOUNT/2) {
        x = 0;
        y = 0;
        z = 1000*0.5+100;
        //z = (POINTCOUNT/8+10)/2*2+100;
      }
      double x2 = cos(angle)*x - sin(angle)*z;
      double z2 = sin(angle)*x + cos(angle)*z;
      x = x2;
      z = z2;
      x += xadd;
      y += yadd;
      pointX[i] = x*220/z + 640/2;
      pointY[i] = y*220/z + 480/2;
      //if (i == 0) {kx = pointX[i] + xadd*220/z; ky = pointY[i] + yadd*220/z;} // that +xadd and +yadd is somehow strange
      if (i == 0 || i == POINTCOUNT/2) {kx[i*2/POINTCOUNT] = pointX[i]; ky[i*2/POINTCOUNT] = pointY[i];} // that +xadd and +yadd is somehow strange
    }
  }
}

void paintPoints() {
  for (int i = 0; i < POINTCOUNT; i++) {
    setPixel(pointX[i], pointY[i], rand() & 15);
  }
}

void paintPoint(int px, int py, int c) {
  for (int y = -1; y <= 1; y++) {
    for (int x = -1; x <= 1; x++) {
      setPixel(px + x, py + y, c);
    }
  }
}

int main(int argc, const char *argv[]) {

  unsigned short mode = getBiosGraphicsMode();

  setBiosGraphicsMode(0x12);
  clearScreenRam();
   
  initPoints();
  paintPoints();

  double xp, yp, w;
  w = 0; xp = 0; yp = 0;
  for (int i = 0; i < POINTCOUNT; i++) {
    xp += pointX[i]; yp += pointY[i]; w++;
  }
  if (w != 0) {xp /= w; yp /= w;}
  double cx = xp,cy = yp;
  cx = kx[0]; cy = ky[0];

  double pX = xp; double pY = yp;

  int frameCount = 0;
  while(1) {
    paintPoint(pX, pY, (frameCount/16) & 15);
    for (int u = 0; u < 2; u++) for (int j = -10; j <= 10; j++) {paintPoint(kx[u]+j, ky[u], 15);paintPoint(kx[u], ky[u]+j, 15);}
    static double l = 1;
    static double r = 1;
    w = 0; xp = 0; yp = 0;
    for (int i = 0; i < POINTCOUNT; i++) {
      double xd = pointX[i] - pX;
      double yd = pointY[i] - pY;
      double d = sqrt(xd*xd+yd*yd);
      if (d < 200) {
        xd = xd / (fabs(xd)+l);
        yd = yd / (fabs(yd)+l);
        xp += xd; yp += yd; w++;
      }
    }
    if (w != 0) {xp /= w; yp /= w;}

    pX += xp*r; pY += yp*r;
    double px2 = pX, py2 = pY;


    if (kbhit()) {
      char k = getch();
      if (k == 'a') break;
      if (k == 'b') l++;
      if (k == 'c') r*=-1;
      pX = rand() % 640;
      pY = rand() % 480;
    }
    frameCount++;
  }

  setBiosGraphicsMode(mode);

  return 0;
}
