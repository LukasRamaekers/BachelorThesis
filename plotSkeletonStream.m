function plotSkeletonStream(fname)

    listing = dir(fname);
    DateNumberOld = listing.datenum; 
    
    counter = 1
    
    while counter<100
        counter = counter+1;
        DateNumberOld = GetFileTime(fname, DateNumberOld);
    end

end

      function DateNumberNew = GetFileTime(fname, DateNumberOld)

          DateNumberNew=DateNumberOld;   
          while(DateNumberNew==DateNumberOld) 
              listing = dir(fname);
              assert(numel(listing) == 1, 'No such file: %s', fname);
              modTime = listing.datenum; 
              DateNumberNew = listing.date;
          end
            disp(now())
      end

