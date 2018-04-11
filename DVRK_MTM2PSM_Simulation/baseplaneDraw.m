function baseplaneDraw
x=-0.4:0.8:0.4;
y=x;
[x,y]=meshgrid(x,y)
z=x*0;
surf(x,y,z)
alpha(.5)