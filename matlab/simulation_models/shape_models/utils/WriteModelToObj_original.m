function WriteModelToObj(obj, filename, varargin)
% write - saves a triangulated mesh struct (obj) as a wavefront obj
% file.

% Giacomo Battaglia, Politecnico di Milano

% ------------- BEGIN CODE --------------

overwrite = false;
for k=1:2:length(varargin)
    switch lower(varargin{k})
        case 'overwrite'
            overwrite = logical(varargin{k+1});
        otherwise
            error('Unrecognized command: %s', varargin{k});
    end
end

% write .obj file
filename = char(lower(filename));
if ~endsWith(filename,'.obj')
    obj_filename=strcat(filename, '.obj');
end

if ~overwrite
    obj_filename_0 = strrep(obj_filename,'.obj','');
    cc=1;
    while isfile(obj_filename)
        obj_filename = [obj_filename_0, '_', num2str(cc,'%d'), '.obj'];
        cc = cc+1;
    end
    clear obj_filename_0 cc
end

has_vt = isfield(obj, 'vt');
has_vn = isfield(obj, 'vn');
has_f_vt = isfield(obj.f, 'vt');
has_f_vn = isfield(obj.f, 'vn');

fid = fopen(obj_filename,'w'); % write new file
fprintf(fid, '# VBN-NAV Generated OBJ File\n');
fprintf(fid, '# Giacomo Battaglia, Politecnico di Milano\n');
if isfield(obj.meta, 'mtllib')
    fprintf(fid, 'mtllib %s\n', obj.meta.mtllib);
end

if obj.meta.count == 0
    obj.meta.o0.name = strrep(filename,'.obj','');
    write_object(fid, obj.meta.o0, obj);
    fclose(fid);
    return;
end

for I=1:obj.meta.count
    write_object(fid, obj.meta.(sprintf('o%d',I)), obj);
end
fclose(fid);

    function write_object(fid, oX, obj)

        fprintf(fid,'o %s\n', oX.name);

        fprintf(fid,['v',repmat(' %.6f',1,size(obj.v,2)),'\n'], obj.v(L(oX.idx_v),:)');
        if has_vt
            fprintf(fid, ['vt',repmat(' %.6f',1,size(obj.vt,2)),'\n'], obj.vt(L(oX.idx_vt),:)');
        end
        if has_vn
            fprintf(fid,'vn %.4f %.4f %.4f\n', obj.vn(L(oX.idx_vn),:)');
        end

        if isfield(oX,'mtl')
            fprintf(fid,'usemtl %s\n', oX.mtl);
        else
            fprintf(fid,'usemtl mat_1\n');
        end
        if isfield(oX,'s')
            fprintf(fid,'s %s\n', oX.s);
        else
            fprintf(fid,'s 1\n');
        end

        f_fmt = ' %d';
        f_mat = [obj.f.v(L(oX.idx_f),:)];
        f_idx = 1;
        if has_f_vt
            f_fmt = [f_fmt, '/%d'];
            f_mat = [f_mat, obj.f.vt(L(oX.idx_f),:)];
            f_idx = f_idx + 1;
        end
        if has_f_vn
            if ~has_f_vt
                f_fmt = [f_fmt, '/'];
            end
            f_fmt = [f_fmt, '/%d'];
            f_mat = [f_mat, obj.f.vn(L(oX.idx_f),:)];
            f_idx = f_idx + 1;
        end
        f_idxv = 1:size(obj.f.v,2);
        fprintf(fid,['f',repmat(f_fmt,1,size(obj.f.v,2)),'\n'], ...
            f_mat(:, reshape([f_idxv; f_idxv+f_idx; f_idxv+2*f_idx], 1, []))');

        function idxl = L(idx)
            idxl = idx(1):idx(2);
        end

    end



end



% ------------- END OF CODE --------------
