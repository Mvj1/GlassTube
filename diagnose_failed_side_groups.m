function diagnostics = diagnose_failed_side_groups(outputDir)
rootDir=fileparts(mfilename('fullpath'));
if nargin<1, outputDir=fullfile(rootDir,'results','robustness_review'); end
cfg=glass_tube_default_config();
specs={...
    'IAV2_044-064','ImagesAndVideos2',44,64;...
    'IAV2_066-086','ImagesAndVideos2',66,86;...
    'IAV2_091-111','ImagesAndVideos2',91,111;...
    'IAV2_133-153','ImagesAndVideos2',133,153;...
    'IAV2_154-174','ImagesAndVideos2',154,174};
rows=repmat(struct(),size(specs,1),1);
for k=1:size(specs,1)
    name=string(specs{k,1}); d=fullfile(rootDir,'raredata',specs{k,2});
    fs=dir(fullfile(d,'*.bmp')); keep=false(numel(fs),1); suffix=nan(numel(fs),1);
    for j=1:numel(fs)
        token=regexp(fs(j).name,'-(\d+)\.bmp$','tokens','once');
        if isempty(token),continue;end
        suffix(j)=str2double(token{1}); keep(j)=suffix(j)>=specs{k,3}&&suffix(j)<=specs{k,4};
    end
    fs=fs(keep); suffix=suffix(keep); [~,o]=sort(suffix); fs=fs(o);
    scfg=struct('imageDir',d,'extension',{{fs.name}},'rotationAngle',cfg.stitch.rotationDeg,...
        'nominal',cfg.stitch.nominalStepPx,'search',[180 12],'downsample',cfg.stitch.registrationScale,...
        'registration',struct('rowRange',cfg.stitch.registrationRowFraction,'templateWidth',cfg.stitch.templateWidthPx),...
        'quality',struct('minPeak',.85,'minPSR',1.8,'onFailure','nominal','psrExclusionRadius',2),'verbose',false);
    [strip,~,q,~]=stitch_tube_strip(scfg);
    gray=im2single(strip); r=cfg.side.roiRect; x1=floor(r(1))+1;y1=floor(r(2))+1;
    x2=min(size(gray,2),x1+round(r(3))-1);y2=min(size(gray,1),y1+round(r(4))-1);
    roi=gray(y1:y2,x1:x2); cols=round(linspace(max(1,round(.08*size(roi,2))),min(size(roi,2),round(.92*size(roi,2))),min(401,size(roi,2))));
    p=imgaussfilt(median(roi(:,cols),2),2); g=gradient(p); mid=round(numel(g)/2);
    [topPos,tp]=max(g(2:mid-1));tp=tp+1; [topNeg,tn]=min(g(2:mid-1));tn=tn+1;
    [botPos,bp]=max(g(mid+1:end-1));bp=bp+mid; [botNeg,bn]=min(g(mid+1:end-1));bn=bn+mid;
    [~,ord]=sort(abs(g),'descend'); peaks=nan(1,12); peakCount=0; for idx=ord', if peakCount==0 || all(abs(idx-peaks(1:peakCount))>8), peakCount=peakCount+1; peaks(peakCount)=idx; end; if peakCount>=12,break;end;end; peaks=peaks(1:peakCount)
    out=fullfile(outputDir,name+'_raw_full.jpg'); scale=min(1,5200/size(gray,2)); imwrite(imresize(strip,scale,'bilinear'),out,'Quality',92);
    rows(k).Group=string(name);rows(k).TopPositiveRow=tp;rows(k).TopPositiveGrad=topPos;rows(k).TopNegativeRow=tn;rows(k).TopNegativeGrad=topNeg;
    rows(k).BottomPositiveRow=bp;rows(k).BottomPositiveGrad=botPos;rows(k).BottomNegativeRow=bn;rows(k).BottomNegativeGrad=botNeg;
    rows(k).PeakRows=string(mat2str(sort(peaks)));rows(k).MinPeak=min(q.peak);rows(k).MinPSR=min(q.psr);rows(k).Fallbacks=q.numFallbacks;rows(k).RawFile=string(out);
    fprintf('%s top +%d(%.4g) -%d(%.4g), bottom +%d(%.4g) -%d(%.4g), peaks %s\n',name,tp,topPos,tn,topNeg,bp,botPos,bn,botNeg,mat2str(sort(peaks)));
    clear strip gray roi;
end
diagnostics=struct2table(rows);writetable(diagnostics,fullfile(outputDir,'failed_group_diagnostics.csv'));
end


