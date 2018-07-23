mom_file = fopen('/Users/karanjitcheema/Desktop/RA_Work/reunion_1_new/centres_mother.txt', 'r');
baby_file = fopen('/Users/karanjitcheema/Desktop/RA_Work/reunion_1_new/centres_mother.txt', 'r');
mother = struct();
baby = struct([]);
i=1;
while(~feof(mom_file))
  A = fscanf(mom_file,'%f %f %f %f %f %f %f %f', 8);
    mother(i,1).value  = A(1);
    mother(i,2).value  = A(2);
    mother(i,3).value  = A(3);
    mother(i,4).value  = A(4);
    mother(i,5).value  = A(5);
    mother(i,6).value  = A(6);
    mother(i,7).value  = A(7);
    mother(i,8).value  = A(8);
    i = i+1;
end
[m,n] = arrayfun(@(x)find(x.2.value==20000),mother,'uniformoutput',false)