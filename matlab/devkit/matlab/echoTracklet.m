function echoTracklet( tracklet )
%ECHOTRACKLET prints contents of tracklet to cmd window

    fprintf('\n\tobjectType: %s\n', tracklet.objectType);
    fprintf('\th: %f, w: %f, l: %f\n', tracklet.h, tracklet.w, tracklet.l);
    fprintf('\tfirst_frame: %d\n', tracklet.first_frame);
    fprintf('\tn poses: %d\n', size(tracklet.poses,2));
    
end

