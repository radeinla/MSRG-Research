function r_room = generate_r_room()
    r_room = zeros(100, 100, 'uint8');
    r_room(:,:) = 255; % set all to white
    r_room(1:5, :) = 0;
    r_room(1:70,1:5) = 0;
    r_room(:, 96:100) = 0;
    r_room(31:70,41:60) = 0;
end