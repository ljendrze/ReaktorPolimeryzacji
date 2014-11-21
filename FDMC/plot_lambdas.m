fileList = cell(4,1);

fileList{1} = '10k_lambda_adjusting.mat';
fileList{2} = '20k_lambda_adjusting.mat';
fileList{3} = '30k_lambda_adjusting.mat';
fileList{4} = '40k_lambda_adjusting.mat';

precisionLB = [ 0.99*10000, 0.99* 20000, 0.99*30000, 0.99*40000 ];
precisionUB = [ 1.01*10000, 1.01* 20000, 1.01*30000, 1.01*40000 ];

for k = 1 : length( fileList )
   load( fileList{k} );
   n = size( results, 1 );
   colors = distinguishable_colors( n );
   
   figure( (k-1)*2 + 1);
   hold on;
   labels = cell( n,1 );
   for i = 1 : n
      temp = results{ i, 2 };
      plot( temp, 'Color', colors(i,:) );
      labels{i} = num2str( results{ i, 1 }, '%10.2e\n' );
   end
   grid on;
   legend( labels );
   plot( [1, size(temp,1)], [precisionUB(k), precisionUB(k)], '--k');
   plot( [1, size(temp,1)], [precisionLB(k), precisionLB(k)], '--k');
   
   
   figure( k*2 );
   hold on;
   labels = cell( n,1 );
   for i = 1 : n
      temp = results{ i, 3 };
      stairs( temp, 'Color', colors(i,:) );
      labels{i} = num2str( results{ i, 1 }, '%10.2e\n' );
   end
   grid on;
   legend( labels );
end
