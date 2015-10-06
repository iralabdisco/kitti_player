function posesVec = extractPoses( poses )
% EXTRACTPOSES extracts poses from subtree of labels.xml

    nPoses = ... % number of annotated poses
        str2double(poses.getElementsByTagName('count').item(0).getTextContent);

    item_version = ... % meta-data
        str2double(poses.getElementsByTagName('item_version').item(0).getTextContent);
    
    posesVec = zeros(14,nPoses);
    
    
    poseChildren = poses.getElementsByTagName('item');
    
    for poseIdx = 0:poseChildren.getLength-1 % all children of 'poses'
        
        p = poseChildren.item(poseIdx);
        
        % translation
        posesVec(1,poseIdx+1) = str2double(p.getElementsByTagName('tx').item(0).getTextContent);
        posesVec(2,poseIdx+1) = str2double(p.getElementsByTagName('ty').item(0).getTextContent);
        posesVec(3,poseIdx+1) = str2double(p.getElementsByTagName('tz').item(0).getTextContent);
        
        % rotation
        posesVec(4,poseIdx+1) = str2double(p.getElementsByTagName('rx').item(0).getTextContent);
        posesVec(5,poseIdx+1) = str2double(p.getElementsByTagName('ry').item(0).getTextContent);
        posesVec(6,poseIdx+1) = str2double(p.getElementsByTagName('rz').item(0).getTextContent);
        
        % state
        posesVec(7,poseIdx+1) = str2double(p.getElementsByTagName('state').item(0).getTextContent);
        
        % occlusion
        posesVec(8,poseIdx+1) = str2double(p.getElementsByTagName('occlusion').item(0).getTextContent);
        posesVec(9,poseIdx+1) = str2double(p.getElementsByTagName('occlusion_kf').item(0).getTextContent);
        
        % truncation
        posesVec(10,poseIdx+1) = str2double(p.getElementsByTagName('truncation').item(0).getTextContent);
        
        % averaged mechanical turk
        posesVec(11,poseIdx+1) = str2double(p.getElementsByTagName('amt_occlusion').item(0).getTextContent);
        posesVec(12,poseIdx+1) = str2double(p.getElementsByTagName('amt_occlusion_kf').item(0).getTextContent);
        posesVec(13,poseIdx+1) = str2double(p.getElementsByTagName('amt_border_l').item(0).getTextContent);
        posesVec(14,poseIdx+1) = str2double(p.getElementsByTagName('amt_border_r').item(0).getTextContent);
        posesVec(15,poseIdx+1) = str2double(p.getElementsByTagName('amt_border_kf').item(0).getTextContent);
    end
    
    
    %% plausibility check
    if nPoses ~= poseIdx+1
        fprintf(2,'number of poses (%d) does not match count (%d)!',nPoses,poseIdx+1);
    end

end