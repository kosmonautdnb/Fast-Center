
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

double point2X[POINTCOUNT];
double point2Y[POINTCOUNT];

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

void setPixel0(int x, int y, int col) {
  if ((unsigned int)x < 640 && (unsigned int)y < 480) {
    for (int j = 0; j < 4; j++) {
      outp(0x3c4, 2);
      outp(0x3c5, 1<<j);
      int b = col & (1<<j) ? 1 : 0;
      if (b) screen[y*(640/8)+x/8+j*640/8*480] |= 1<<(7-(x & 7));
      if (!b) screen[y*(640/8)+x/8+j*640/8*480] &= 255-(1<<(7-(x & 7)));
    }
  }
}

void setPixel2(int x, int y, int col) {
  for (int y2 = -1; y2 <= 1; y2++) {
    for (int x2 = -1; x2 <= 1; x2++) {
      setPixel0(x+x2,y+y2,col);
    }
  }
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

double kx[4],ky[4];

void initPoints() {
  bool clusters = false;
  if (clusters) {
    double cx,cy,cz;
    double mX=0,mY=0,mZ=0,mW=0;
    double addX = 50;
    double addY = 50;
    for (int i = 0; i < POINTCOUNT; i++) {
      if ((i % CLUSTERSIZE) == 0) {
        cx = (rand() % 101)-50 + addX;
        cy = (rand() % 101)-50 + addY;
        cz = 100 + (rand() % 1001);
      }
      double x = cx+(rand() % 20)-10;
      double y = cy+(rand() % 20)-10;
      double z = cz+(rand() % 20)-10;
      mX += x;
      mY += y;
      mZ += z;
      mW++;
      pointX[i] = x*220/z + 640/2;
      pointY[i] = y*220/z + 480/2;
    }
    mX /= mW;
    mY /= mW;
    mZ /= mW;
    mX = mX*220/mZ + 640/2;
    mY = mY*220/mZ + 480/2;
    for (int u = 0; u < 4; u++) {kx[u] = mX; ky[u] = mY;}
  } else {
    // anscheinend bekommt man so den mittelpunkt einer 3dimensionalen punktwolke (alles in 2d)
    double xadd = 30*1;
    double yadd = 30*1;
    double angle = 40*3.14159/180.0;
    double angle2 = 5*3.14159/180.0;
    for (int i = 0; i < POINTCOUNT; i++) {
      if (i == POINTCOUNT/2) angle = -angle;
      double x = (rand() % 101)-50;
      double y = (rand() % 101)-50;
      //double d = rand() % 50;
      //double a = (rand() & 1023)*3.1415*2/1024;
      //x = sin(a)*d;
      //y = cos(a)*d;
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
      if (i == 1 || i == POINTCOUNT/2+1) {
        x = 0;
        y = 0;
        z = 10000+100;
        //z = (POINTCOUNT/8+10)/2*2+100;
      }
      double x2 = cos(angle)*x - sin(angle)*z;
      double z2 = sin(angle)*x + cos(angle)*z;
      x = x2;
      z = z2;

      double y3 = cos(angle2)*y + sin(angle2)*x;
      double x3 = -sin(angle2)*y + cos(angle2)*x;
      y = y3;
      x = x3;

      x += xadd;
      y += yadd;
      pointX[i] = x*220/z + 640/2;
      pointY[i] = y*220/z + 480/2;
      //if (i == 0) {kx = pointX[i] + xadd*220/z; ky = pointY[i] + yadd*220/z;} // that +xadd and +yadd is somehow strange
      if (i == 0 || i == POINTCOUNT/2) {kx[i*2/POINTCOUNT] = pointX[i]; ky[i*2/POINTCOUNT] = pointY[i];} // that +xadd and +yadd is somehow strange
      if (i == 1 || i == POINTCOUNT/2 + 1) {kx[(i-1)*2/POINTCOUNT+2] = pointX[i]; ky[(i-1)*2/POINTCOUNT+2] = pointY[i];} // that +xadd and +yadd is somehow strange
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

float frand() {
  return (float)rand() / RAND_MAX;
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
    for (int u = 0; u < 4; u++) for (int j = -10; j <= 10; j++) {paintPoint(kx[u]+j, ky[u], 15);paintPoint(kx[u], ky[u]+j, 15);}
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
      if (k == 'd') {
        double cx2 = pX, cy2 = pY;
        int ri = 0;
        for (double a = 0; a < 360.0; a += 45.0/8.0) {
          double xd3 = sin(a*3.14159/180.0);
          double yd3 = cos(a*3.14159/180.0);
          double w,xp,yp,pX,pY;
          w = 0; xp = 0; yp = 0;
          pX = cx2; pY = cy2;
          //double s2 = 0.3; int steps = 250; double t = 10;
          //double s2 = 0.1; int steps = 10; double t = 1;
          //double s2 = 0.01; int steps = 1; double t = 1;
          double s2 = 0.1; int steps = 1; double t = 0.1;
          for (int l = 0; l < steps; l++) {
            for (int i = 0; i < POINTCOUNT; i++) {
              double xd2 = pointX[i] - cx2;
              double yd2 = pointY[i] - cy2;
              double d2 = xd3*xd2+yd3*yd2;
              if (d2 >= 0) {
                double xd = pointX[i] - pX;
                double yd = pointY[i] - pY;
                double d = sqrt(xd*xd+yd*yd);
                if (d < 200) {
                  xd = xd / (fabs(xd)+l);
                  yd = yd / (fabs(yd)+l);
                  xp += xd; yp += yd; w++;
                }
              }
            }
            if (w != 0) {xp /= w; yp /= w;}
            pX += xp*t; pY += yp*t;
          }
          double t2 = 10.0*250/steps*s2;
          point2X[ri] = (pX-cx2)*t2+cx2;
          point2Y[ri] = (pY-cy2)*t2+cy2;
          paintPoint(point2X[ri], point2Y[ri], 15);
          ri++;
        }
/*
        int found = 0;
        double ld = 0;
        for (int n = 1; n < ri-1; n++) {
          double dx = point2X[n+1] - point2X[n-1];
          double dy = point2Y[n+1] - point2Y[n-1];
          double d = dx*dx+dy*dy;
          if (n == 1 || d < ld) {
            ld = d;
            found = n;
          }
        }

        double centerX = point2X[found];
        double centerY = point2Y[found];

        double x = cx2;
        double y = cy2;
        double xa = (centerX-cx2)/100.0;
        double ya = (centerY-cy2)/100.0;
        int steps = 5000;
        x -= xa * steps/2;
        y -= ya * steps/2;
        for (int m = 0; m < steps; m++) {
          paintPoint(x, y, 2);
          x += xa;
          y += ya;
        }
*/
      }

      pX = rand() % 640;
      pY = rand() % 480;
    }
    frameCount++;
  }


  while(!kbhit()) {
    double pX = 640/2, pY = 480/2;
    memset(screen,0,sizeof(screen));

    srand(0);
    static float angle2 = 0; angle2 += 0.1;
    static float angle = 0; angle += 0.04;
    double mX = 0, mY = 0, mZ = 0, mW = 0;
    double cx = 0, cy = 0, cz = 0;
    double cX = 0, cY = 0;
    double sx = 1, sy = 1, sz = 1;
    {for (int i = 0; i < POINTCOUNT; i++) {
      if ((i % CLUSTERSIZE) == 0) {
        cx = frand()*100-50; // the algorithm works better with sort of quantized (to a grid) points
        cy = frand()*100-50;
        cz = 100 + frand()*1001;
        sx = frand()*2+1;
        sy = frand()*2+1;
        sz = frand()*2+1;
      }
      double x = cx+((frand() * 20)-10)*sx;
      double y = cy+((frand() * 20)-10)*sy;
      double z = cz+((frand() * 20)-10)*sz;
      x += 100;
      y += 100;
      z += 100;

      z -= 550;
      double x2 = cos(angle)*x - sin(angle)*z;
      double z2 = sin(angle)*x + cos(angle)*z;
      x = x2;
      z = z2;
      z += 550;
      if (z < 100) z = 100;

      double y3 = cos(angle2)*y + sin(angle2)*x;
      double x3 = -sin(angle2)*y + cos(angle2)*x;
      y = y3;
      x = x3;

      mX += x;
      mY += y;
      mZ += z;
      mW++;
      pointX[i] = x*220/z + 640/2;
      pointY[i] = y*220/z + 480/2;
      cX += pointX[i];
      cY += pointY[i];
    }}
    mX /= mW;
    mY /= mW;
    mZ /= mW;
    mX = mX*220/mZ + 640/2;
    mY = mY*220/mZ + 480/2;
    cX /= mW;
    cY /= mW;

    {for (int i = 0; i < POINTCOUNT; i++) {
      setPixel0(pointX[i], pointY[i], rand() & 15);
    }}

    {for (int j = -10; j <= 10; j++) {setPixel2(cX+j, cY, 1);setPixel2(cX, cY+j, 1);}} // 2d average
    {for (int j = -10; j <= 10; j++) {setPixel2(mX+j, mY, 4);setPixel2(mX, mY+j, 4);}} // 3d center

    // the algorithm
    {for (int j = 0; j < 1000; j++) {
      w = 0; xp = 0; yp = 0;
      for (int i = 0; i < POINTCOUNT; i++) {
        double xd = pointX[i] - pX;
        double yd = pointY[i] - pY;
        if (true) {
          xd = xd / (fabs(xd)+1);
          yd = yd / (fabs(yd)+1);
        } else {
          double l = sqrt(xd*xd+yd*yd);
          xd = xd / (l+1);
          yd = yd / (l+1);
        }
        xp += xd; yp += yd; w++;
      }
      if (w != 0) {xp /= w; yp /= w;}
      pX += xp; pY += yp;
      setPixel0(pX,pY,15);
    }}

    // this paints the "Sichtachse" notice that the inferior and slower "sqrt()" function is used here
    if (true) {
      double cx2 = pX, cy2 = pY;
      double ld = 0, ax = 0, ay = 0;
      for (double a = 0; a < 360.0; a += 2.0) {
        double xd3 = sin(a*3.14159/180.0);
        double yd3 = cos(a*3.14159/180.0);
        double w,xp,yp,pX,pY;
        w = 0; xp = 0; yp = 0;
        pX = cx2; pY = cy2;
        for (int i = 0; i < POINTCOUNT; i++) {
          double xd2 = pointX[i] - cx2;
          double yd2 = pointY[i] - cy2;
          double d2 = xd3*xd2+yd3*yd2;
          if (d2 >= 0) {
            double xd = pointX[i] - pX;
            double yd = pointY[i] - pY;
            double d = sqrt(xd*xd+yd*yd);
            xd = xd / (d+1);
            yd = yd / (d+1);
            xp += xd; yp += yd; w++;
          }
        }
        if (w != 0) {xp /= w; yp /= w;}
        double l = sqrt(xp*xp+yp*yp);
        if (l > ld) {
          ax = xp;
          ay = yp;
          ld = l;
        }
      }
      double l = sqrt(ax*ax+ay*ay);
      if (l != 0) {ax /= l; ay /= l;}
      for (int j = -sqrt(640*480); j < sqrt(640*480); j++) {
        cx2 = pX+ax*j;
        cy2 = pY+ay*j;
        xp = 0; yp = 0; w = 0;
        for (int i = 0; i < POINTCOUNT; i++) {
          double xd = pointX[i] - cx2;
          double yd = pointY[i] - cy2;
          double d = sqrt(xd*xd+yd*yd);
          xd = xd / (d+1);
          yd = yd / (d+1);
          xp += xd; yp += yd; w++;
        }
        if (w != 0) {xp /= w; yp /= w;}

        double k = sqrt(xp*xp+yp*yp);
        if (j < 0) {k *= -1;}
        setPixel0(cx2,cy2,(int)(k*100) & 15);
      }
    }

    {for (int j = -10; j <= 10; j++) {setPixel2(pX+j, pY-j, 15);setPixel2(pX+j, pY+j, 15);}}

    for (int y = 0; y < 480; y++) {
      for (int j = 0; j < 4; j++) {
        outp(0x3c4, 2);
        outp(0x3c5, 1<<j);
        for (int x = 0; x < 640; x++) {
          *(unsigned char *)(0xa0000+y*(640/8)+x/8) = screen[y*(640/8)+x/8+j*640/8*480];
        }
      }
    }
  }
  getch();

  for (int k = 0; k < 2; k++) {
    for (int y = 0; y < 480; y++) {
      for (int x = 0; x < 640; x++) {
        w = 0; xp = 0; yp = 0;
        for (int i = 0; i < POINTCOUNT; i++) {
          double xd = pointX[i] - x;
          double yd = pointY[i] - y;
          if (k == 0) {
            xd = xd / (fabs(xd)+1);
            yd = yd / (fabs(yd)+1);
          } else {
            double l = sqrt(xd*xd+yd*yd);
            xd = xd / (l+1);
            yd = yd / (l+1);
          }
          xp += xd; yp += yd; w++;
        }
        if (w != 0) {xp /= w; yp /= w;}
        double l = sqrt(xp * xp + yp * yp);
        setPixel(x,y,(int)(l*100) & 15);
      }
      if (kbhit()) {getch(); break;}
    }
    while (!kbhit()) {;}
    getch();
  }

  setBiosGraphicsMode(mode);

  return 0;
}
