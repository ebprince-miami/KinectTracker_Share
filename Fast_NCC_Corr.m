%--------- Multi/Single Channel NCC Correlation -----------------
% Fast implementation based on Matlab normxcorr2 implementation --------
function [ncc ypeak xpeak] = Fast_NCC_Corr(I,T);
% I - Image
% T - Template

   [M N Ch] = size(I);
   [m n Ch] = size(T);

   Channel = size(I,3);
   T = double(T); I = double(I); % IMPORTANT *****
   mn = m*n;
   MN = M*N;
   
switch Channel
    
case 1 % Mono Channel Image - This is the standard normxcorr2 implementation

    xcorr_TI = xcorr2_fast(T,I);

    % Correction of normx2corr: discarding the border columns and  raws due to the paddarray used
    % in the convolution process of xcorr2_fast
    xcorr_TI = xcorr_TI(m:end-m,n:end-n);

   local_sum_I = local_sum(I,m,n);
   local_sum_I2 = local_sum(I.*I,m,n);
   diff_local_sums = ( local_sum_I2 - (local_sum_I.^2)/mn );
   denom_I = sqrt( max(diff_local_sums,0) ); 
   cc_d = sqrt(mn-1)*std(T(:))*denom_I; %sqrt(MN-1)*std(I(:));
   cc_n = (xcorr_TI - local_sum_I*sum(T(:))/mn );
%   ncc = cc_n./cc_d(round(m/2):M-round(m/2),round(n/2):N-round(n/2));
   
   ncc = zeros(size(cc_n));
   tol = 1000*eps( max(abs(cc_d(:))) );
   i_nonzero = find(cc_d > tol);
   ncc(i_nonzero) = cc_n(i_nonzero) ./ cc_d(i_nonzero);
   
    [max_cc, imax] = max(ncc(:));
    [ypeak, xpeak] = ind2sub(size(ncc),imax(1));
    return;


case 3 % 3 Channel Image
    
   IR = I(:,:,1); IG = I(:,:,2); IB = I(:,:,3);
   TR = T(:,:,1); TG = T(:,:,2); TB = T(:,:,3);
   
   xcorr_TIR = xcorr2_fast(TR,IR);
   xcorr_TIG = xcorr2_fast(TG,IG);
   xcorr_TIB = xcorr2_fast(TB,IB);
% My Correction: Concatination of the borders due to the paddarray used
% in the convolution process of xcorr2_fast
   xcorr_TIR = xcorr_TIR(m:end-m,n:end-n);
   xcorr_TIG = xcorr_TIG(m:end-m,n:end-n);
   xcorr_TIB = xcorr_TIB(m:end-m,n:end-n);

   local_sum_R = local_sum(IR,m,n);
   local_sum_R2 = local_sum(IR.*IR,m,n);
   
   local_sum_G = local_sum(IG,m,n);
   local_sum_G2 = local_sum(IG.*IG,m,n);

   local_sum_B = local_sum(IB,m,n);
   local_sum_B2 = local_sum(IB.*IB,m,n);

   
   cc_n = (xcorr_TIR - local_sum_R*sum(TR(:))/mn ...
         + xcorr_TIG - local_sum_G*sum(TG(:))/mn ...
         + xcorr_TIB - local_sum_B*sum(TB(:))/mn);

   diff_local_sumsR = ( local_sum_R2 - (local_sum_R.^2)/mn );
   diff_local_sumsG = ( local_sum_G2 - (local_sum_G.^2)/mn );
   diff_local_sumsB = ( local_sum_B2 - (local_sum_B.^2)/mn );
      
   denom_R = sqrt( max(diff_local_sumsR,0) );
   denom_G = sqrt( max(diff_local_sumsG,0) );
   denom_B = sqrt( max(diff_local_sumsB,0) );

   cc_d = sqrt(mn-1)*(std(TR(:))+std(TG(:))+std(TB(:)))*(denom_R+denom_G+denom_B);
     
% We know denom_T~=0 from input parsing;
% so denom is only zero where denom_A is zero, and in 
% these locations, C is also zero.
   
   ncc = zeros(size(cc_n));
   tol = 1000*eps( max(abs(cc_d(:))) );
   i_nonzero = find(cc_d > tol);
   ncc(i_nonzero) = cc_n(i_nonzero) ./ cc_d(i_nonzero);
  
%   ncc = cc_n./cc_d;

end % switch
%************** This Linear Filter could be essential  ****************
%sig0 = 1.5; hsize0 = 5;
%gaus = fspecial('gaussian',hsize0,sig0);
%ncc = imfilter(ncc,gaus,'symmetric','same','conv');
 
[max_S, imax] = max(ncc(:));
[ypeak, xpeak] = ind2sub(size(ncc),imax);

%------------------------------------------------
function local_sum_A = local_sum(A,m,n)

% We thank Eli Horn for providing this code, used with his permission,
% to speed up the calculation of local sums. The algorithm depends on
% precomputing running sums as described in "Fast Normalized
% Cross-Correlation", by J. P. Lewis, Industrial Light & Magic.
% http://www.idiom.com/~zilla/Papers/nvisionInterface/nip.html

B = padarray(A,[m n]);
s = cumsum(B,1);
c = s(1+m:end-1,:)-s(1:end-m-1,:);
s = cumsum(c,2);
local_sum_A = s(:,1+n:end-1)-s(:,1:end-n-1);
% My Extension - Concatenation of the paddings
local_sum_A  = local_sum_A(m:end-m,n:end-n);

%------------------------------------------
function cross_corr = xcorr2_fast(T,A)

T_size = size(T);
A_size = size(A);
outsize = A_size + T_size - 1;

% figure out when to use spatial domain vs. freq domain
conv_time = time_conv2(T_size,A_size); % 1 conv2
fft_time = 3*time_fft2(outsize); % 2 fft2 + 1 ifft2

if (conv_time < fft_time)
    cross_corr = conv2(rot90(T,2),A);
else
    cross_corr = freqxcorr(T,A,outsize);
end
%--------------------------------------------
%-------------------------------
% Function  freqxcorr
%
function xcorr_ab = freqxcorr(a,b,outsize)
  
% calculate correlation in frequency domain
Fa = fft2(rot90(a,2),outsize(1),outsize(2));
Fb = fft2(b,outsize(1),outsize(2));
xcorr_ab = real(ifft2(Fa .* Fb));


%-------------------------------
% Function  time_conv2
%
function time = time_conv2(obssize,refsize)

% time a spatial domain convolution for 10-by-10 x 20-by-20 matrices

% a = ones(10);
% b = ones(20);
% mintime = 0.1;

% t1 = cputime;
% t2 = t1;
% k = 0;
% while (t2-t1)<mintime
%     c = conv2(a,b);
%     k = k + 1;
%     t2 = cputime;
% end
% t_total = (t2-t1)/k;

% % convolution time = K*prod(size(a))*prod(size(b))
% % t_total = K*10*10*20*20 = 40000*K
% K = t_total/40000;

% K was empirically calculated by the commented-out code above.
K = 2.7e-8; 
            
% convolution time = K*prod(obssize)*prod(refsize)
time =  K*prod(obssize)*prod(refsize);


%-------------------------------
% Function  time_fft2
%
function time = time_fft2(outsize)

% time a frequency domain convolution by timing two one-dimensional ffts

R = outsize(1);
S = outsize(2);

% Tr = time_fft(R);
% K_fft = Tr/(R*log(R)); 

% K_fft was empirically calculated by the 2 commented-out lines above.
K_fft = 3.3e-7; 
Tr = K_fft*R*log(R);

if S==R
    Ts = Tr;
else
%    Ts = time_fft(S);  % uncomment to estimate explicitly
   Ts = K_fft*S*log(S); 
end

time = S*Tr + R*Ts;

% %-------------------------------
% % Function time_fft
% %
% function T = time_fft(M)

% % time a complex fft that is M elements long

% vec = complex(ones(M,1),ones(M,1));
% mintime = 0.1; 

% t1 = cputime;
% t2 = t1;
% k = 0;
% while (t2-t1) < mintime
%     dummy = fft(vec);
%     k = k + 1;
%     t2 = cputime;
% end
% T = (t2-t1)/k;


%-----------------------------------------------------------------------------
function [T, A] = ParseInputs(varargin)

iptchecknargin(2,2,nargin,mfilename)

T = varargin{1};
A = varargin{2};

iptcheckinput(T,{'logical','numeric'},{'real','nonsparse','2d','finite'},mfilename,'T',1)
iptcheckinput(A,{'logical','numeric'},{'real','nonsparse','2d','finite'},mfilename,'A',2)

checkSizesTandA(T,A)

% See geck 342320. If either A or T has a minimum value which is negative, we
% need to shift the array so all values are positive to ensure numerically
% robust results for the normalized cross-correlation.
A = shiftData(A);
T = shiftData(T);

checkIfFlat(T);

%-----------------------------------------------------------------------------
function B = shiftData(A)

B = double(A);

is_unsigned = isa(A,'uint8') || isa(A,'uint16') || isa(A,'uint32');
if ~is_unsigned
    
    min_B = min(B(:)); 
    
    if min_B < 0
        B = B - min_B;
    end
    
end

%-----------------------------------------------------------------------------
function checkSizesTandA(T,A)

if numel(T) < 2
    eid = sprintf('Images:%s:invalidTemplate',mfilename);
    msg = 'TEMPLATE must contain at least 2 elements.';
    error(eid,'%s',msg);
end

if size(A,1)<size(T,1) || size(A,2)<size(T,2) 
    eid = sprintf('Images:%s:invalidSizeForA',mfilename);
    msg = 'A must be the same size or larger than TEMPLATE.';
    error(eid,'%s',msg);
end

%-----------------------------------------------------------------------------
function checkIfFlat(T)

if std(T(:)) == 0
    eid = sprintf('Images:%s:sameElementsInTemplate',mfilename);
    msg = 'The values of TEMPLATE cannot all be the same.';
    error(eid,'%s',msg);
end
