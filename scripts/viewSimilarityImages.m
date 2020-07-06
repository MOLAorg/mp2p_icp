function [  ] = viewSimilarityImages(i)
%VIEWSIMILARITYIMAGES Debug script for similarity metric method.


I11=load(sprintf('I11_%05i.txt',i));
I12=load(sprintf('I12_%05i.txt',i));
I21=load(sprintf('I21_%05i.txt',i));
I22=load(sprintf('I22_%05i.txt',i));

close;
subplot(2,2,1);imagesc(I11); title('I11');
subplot(2,2,2); imagesc(I12); title('I12');
subplot(2,2,3); imagesc(I21); title('I21');
subplot(2,2,4); imagesc(I22); title('I22');


end

