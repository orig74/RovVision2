//vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
int convertRGBtoBayer(int sx,int sy,unsigned char* in,unsigned char* out)
{
  /*
  Assuming a Bayer filter that looks like this:

  # // 0  1  2  3  4  5
  /////////////////////
  0 // B  G  B  G  B  G
  1 // G  R  G  R  G  R
  2 // B  G  B  G  B  G
  3 // G  R  G  R  G  R
  4 // B  G  B  G  B  G
  5 // G  R  G  R  G  R

  */


  int channel;

  for (int row = 0; row < sy; row++)
  {
    for (int col = 0; col < sx; col++)
    {
      if (row % 2 == 0)
      {
        //even columns and even rows = blue = channel:0
        //even columns and uneven rows = green = channel:1 
        channel = (col % 2 == 0) ? 0 : 1;
      }
      else
      {
        //uneven columns and even rows = green = channel:1
        //uneven columns and uneven rows = red = channel:2 
        channel = (col % 2 == 0) ? 1 : 2;
      }
      
      out[row*sx+col]=in[(row*sx+col)*3+channel];
      
    }
  }
  return 0;
}


int shrinkToRGB(int sx,int sy,unsigned char* in,unsigned char* out)
{
  for (int row = 0; row < sy/2; row++)
    for (int col = 0; col < sx/2; col++)
    {
      int out_idx=(row*sx/2+col)*3;
      int b=in[(row*2)*sx+col*2];
      int g=in[(row*2)*sx+col*2+1];
      int r=in[(row*2+1)*sx+col*2+1];
      out[out_idx]=r;
      out[out_idx+1]=g;
      out[out_idx+2]=b;
    }
  return 0;
}
