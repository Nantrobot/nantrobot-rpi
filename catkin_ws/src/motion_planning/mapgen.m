pkg load image
im=imread('Map.png');
im=im/255;
im=1-im;
se=strel('ball',25,0);
im=imdilate(im,se);
im=imrotate(im,180);
fid=fopen('map.txt','w');
s=size(im);
fprintf(fid,'%d\n',s(1));
fprintf(fid,'%d\n',s(2));
n=s(1)*s(2);
cnt=0;
h=waitbar(0,'Generation fichier txt...');
for i=1:1:s(1)
    for j=1:1:s(2)
        fprintf(fid,'%d',im(i,j));
        fprintf(fid,' ');
        cnt=cnt+1;
        waitbar(cnt/n);
    end
    fprintf(fid,'\n');
end
fclose(fid);
close(h);
