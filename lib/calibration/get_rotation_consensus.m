function [R_ab t_ab s_ab] = get_rotation_consensus(matches,pcda,pcdb,fa,fb)
% This function finds R_ab and t_ab that satisfy pcda = R_ab*pcdb + t_ab;

    xa = round(fa(1,matches(1,:)));
    xb = round(fb(1,matches(2,:)));
    ya = round(fa(2,matches(1,:)));
    yb = round(fb(2,matches(2,:)));

    keep_points = logical(ones(length(xa),1));
    pcda_f = zeros(length(xa),3);
    for i=1:length(xa)
        pcda_f(i,:) = reshape(pcda(ya(i),xa(i),:), [1 3]);
        if isnan(pcda_f(i,1)) || pcda_f(i,3) <= 0
            keep_points(i) = 0;
        end
    end
    
    pcdb_f = zeros(length(xb),3);
    for i=1:length(xb)
        pcdb_f(i,:) = reshape(pcdb(yb(i),xb(i),:), [1 3]);
        if isnan(pcdb_f(i,1)) || pcdb_f(i,3) <= 0
            keep_points(i) = 0;
        end
    end
    
    % Remove nan points
    pcda_f = pcda_f(keep_points,:);
    pcdb_f = pcdb_f(keep_points,:);
    
%     n = 200;
%     R_ab = zeros(3,3,n);
%     idx_all = zeros(n,3);
%     for i=1:n
%         perm = randperm(size(pcda_f,1));
%         idx = perm(1:3);
%         R_ab(:,:,i) = get_rotation_matrix(pcda_f,pcdb_f,idx);
% %         Reg_params(i) = absor(pcda_f(idx,:)',pcdb_f(idx,:)','doScale',1);
%         idx_all(i,:) = idx;
%     end
%     
%     thres = 0.01;
%     votes = zeros(n,1);
%     inliers = zeros(n);
%     parfor i=1:n
%         d = zeros(n,1);
%         for j=1:n
%             d(j) = get_distance(R_ab(:,:,i),R_ab(:,:,j));
% %             d(j) = get_distance(Reg_params(i).R,Reg_params(j).R);
%         end
%         inliers(i,:) = d<thres;
%         votes(i) = sum(inliers(i,:));
%     end
%     [a,b] = sort(votes,'descend');
%     
%     R_ab = R_ab(:,:,b(1));
%     t_ab = pcda_f(idx_all(b(1),1),:)' - R_ab*pcdb_f(idx_all(b(1),1),:)';

%     idx = 1:3;
%     R_ab = get_rotation_matrix(pcda_f,pcdb_f,idx);
%     t_ab = pcda_f(idx(1),:)' - R_ab*pcdb_f(idx(1),:)';
%     s_ab = 1;
%     R_ab = Reg_params(b(1)).R;
%     t_ab = Reg_params(b(1)).t;

%     inl = find(inliers(b(1),:));
%     idx = idx_all(inl,:);
%     idx = idx(:);

    idx = 1:size(pcda_f,1);
    Reg_params = absor(pcdb_f(idx,:)',pcda_f(idx,:)','doScale',0);
    R_ab = Reg_params.R;
    t_ab = Reg_params.t;
    s_ab = Reg_params.s;
end

function R_ab = get_rotation_matrix(p_a,p_b,idx)
    Ma = get_basis_vectors(p_a(idx,:));
    Mb = get_basis_vectors(p_b(idx,:));
    
    R_ab = Ma * Mb';
end

function M = get_basis_vectors(p)
    x = p(2,:) - p(1,:);
    x = x ./ norm(x);
    
    y = p(3,:) - p(1,:) - x*((p(3,:)-p(1,:))*x');
    y = y ./ norm(y);
    
    z = cross(x,y);
    z = z./ norm(z);
    
    M = [x' y' z'];
end

function d = get_distance(R1,R2)
    I = eye(size(R1));
    d = sqrt(trace((I-R1*R2').^2));
end