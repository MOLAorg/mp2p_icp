function [  ] = viewSimilarityImages(i)
%VIEWSIMILARITYIMAGES Debug script for similarity metric method.


I11=load(sprintf('I11_%05i.txt',i));
I12=load(sprintf('I12_%05i.txt',i));
I21=load(sprintf('I21_%05i.txt',i));
I22=load(sprintf('I22_%05i.txt',i));

I1_scores=load(sprintf('I1_scores_%05i.txt',i));
I2_scores=load(sprintf('I2_scores_%05i.txt',i));

close;
subplot(2,3,1);imagesc(I11); axis equal; title('I11');
subplot(2,3,2); imagesc(I12); axis equal; title('I12');
subplot(2,3,3); histogram(I1_scores,'BinLimits',[0.0,1.0]);  
title(sprintf('Scores I11 vs I12 (mean=%.03f)', mean(I1_scores)));

subplot(2,3,4); imagesc(I21); axis equal; title('I21');
subplot(2,3,5); imagesc(I22); axis equal; title('I22');
subplot(2,3,6); histogram(I2_scores,'BinLimits',[0.0,1.0]);
title(sprintf('Scores I21 vs I22 (mean=%.03f)', mean(I2_scores)));


end

