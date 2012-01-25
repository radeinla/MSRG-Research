function r_room = generate_r_room()
    r_room = zeros(10, 10, 'uint8');
    r_room(:,:) = 255; % set all to white
    r_room(1, :) = 0;
    r_room(1:7,1) = 0;
    r_room(:, 10) = 0;
    r_room(4:7,5:6) = 0;
end