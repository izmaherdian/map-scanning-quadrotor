%% ZIGZAG_TRAJECTORY  Generate and animate the zig-zag (boustrophedon) scanning path.
% The drone sweeps along x, steps along y by 'belokan', and returns to the
% start point once the whole rectangular area has been covered.

% Contoh penggunaan
% Input
panjang = 50;         % Panjang lintasan x  
lebar = 55;           % Lebar area pada sumbu y
belokan = 10;         % Jarak antar lintasan y

kecepatan = 5;        % Kecepatan drone
waktu_simulasi = 120; % Waktu simulasi keseluruhan (detik)

% Input posisi awal
posisix_awal = 10;     % Posisi awal pada sumbu x
posisiy_awal = 12;     % Posisi awal pada sumbu y

% Panggil fungsi lintasan_drone untuk menghitung lintasan dan menampilkan animasi
[posisi_terbaru_x, posisi_terbaru_y] = lintasan_drone(panjang, lebar, belokan, kecepatan, waktu_simulasi, posisix_awal, posisiy_awal);

% Fungsi untuk menghasilkan lintasan drone zig-zag
function [posisi_terbaru_x, posisi_terbaru_y] = lintasan_drone(panjang, lebar, belokan, kecepatan, waktu_simulasi, posisix_awal, posisiy_awal)
    % Inisialisasi
    posisi_terbaru_x = posisix_awal;
    posisi_terbaru_y = posisiy_awal;

    % Jumlah lintasan pada sumbu y
    jumlah_belokan = ceil(lebar / belokan);

    % Inisialisasi lintasan
    x_d = posisix_awal;
    y_d = posisiy_awal;
    waktu_d = 0; % Waktu untuk setiap titik lintasan

    % Variabel posisi awal
    x_pos = posisix_awal;
    y_pos = posisiy_awal;
    arah_x = 1; % 1: ke kanan, -1: ke kiri
    waktu_total = 0; % Waktu total lintasan

    % Iterasi untuk membuat lintasan zig-zag
    for i = 1:jumlah_belokan
        % Tentukan arah gerakan pada sumbu x (ke kanan atau ke kiri)
        x_target = x_pos + arah_x * panjang;

        % Buat lintasan pada sumbu x
        t_x = abs(x_target - x_pos) / kecepatan; % waktu tempuh pada x
        t_segment = linspace(waktu_total, waktu_total + t_x, 100);
        x_segment = linspace(x_pos, x_target, 100);
        y_segment = y_pos * ones(size(x_segment));

        % Tambahkan ke lintasan
        x_d = [x_d, x_segment];
        y_d = [y_d, y_segment];
        waktu_d = [waktu_d, t_segment];

        % Perbarui posisi x dan waktu total
        x_pos = x_target;
        waktu_total = waktu_total + t_x;

        % Tentukan posisi y untuk belokan berikutnya
        if i < jumlah_belokan
            y_target = y_pos + belokan;
            t_y = belokan / kecepatan; % waktu tempuh pada y
            t_segment = linspace(waktu_total, waktu_total + t_y, 100);
            x_segment = x_pos * ones(size(t_segment));
            y_segment = linspace(y_pos, y_target, 100);

            % Tambahkan ke lintasan
            x_d = [x_d, x_segment];
            y_d = [y_d, y_segment];
            waktu_d = [waktu_d, t_segment];

            % Perbarui posisi y dan waktu total
            y_pos = y_target;
            waktu_total = waktu_total + t_y;

            % Ganti arah gerakan sumbu x
            arah_x = -arah_x; % Membalik arah (ke kiri atau ke kanan)
        end

        % Cek jika waktu simulasi sudah tercapai
        if waktu_total >= waktu_simulasi
            break;
        end
    end

    % Pastikan mencapai y paling ujung
    if y_pos < lebar + posisiy_awal && waktu_total < waktu_simulasi
        y_target = lebar + posisiy_awal;
        t_y = (y_target - y_pos) / kecepatan;
        t_segment = linspace(waktu_total, waktu_total + t_y, 100);
        x_segment = x_pos * ones(size(t_segment));
        y_segment = linspace(y_pos, y_target, 100);

        % Tambahkan ke lintasan
        x_d = [x_d, x_segment];
        y_d = [y_d, y_segment];
        waktu_d = [waktu_d, t_segment];

        % Perbarui posisi y dan waktu total
        y_pos = y_target;
        waktu_total = waktu_total + t_y;

        % Perbarui arah lintasan sumbu x agar drone terus bergerak
        arah_x = -1 * arah_x; % Membalik arah x agar terus zig-zag
        x_target = x_pos + arah_x * panjang;

        % Buat lintasan pada sumbu x setelah mencapai y ujung
        t_x = abs(x_target - x_pos) / kecepatan; % waktu tempuh pada x
        t_segment = linspace(waktu_total, waktu_total + t_x, 100);
        x_segment = linspace(x_pos, x_target, 100);
        y_segment = y_pos * ones(size(x_segment));

        % Tambahkan lintasan baru
        x_d = [x_d, x_segment];
        y_d = [y_d, y_segment];
        waktu_d = [waktu_d, t_segment];

        % Perbarui posisi x
        x_pos = x_target;
        waktu_total = waktu_total + t_x;
    end

    % Kembali ke titik asal
    if x_pos == 0
        % Jika posisi terakhir di kiri (x = 0), hanya bergerak di sumbu y
        waktu_return = abs(y_pos - posisiy_awal) / kecepatan; % Menghitung waktu untuk kembali ke posisi awal y
        t_segment = linspace(waktu_total, waktu_total + waktu_return, 100);
        x_segment = posisix_awal * ones(size(t_segment)); % Tetap di x = posisix_awal
        y_segment = linspace(y_pos, posisiy_awal, 100); % Gerakan ke posisiy_awal
    else
        % Jika posisi terakhir di kanan (x ≠ 0), bergerak diagonal ke (posisix_awal, posisiy_awal)
        jarak = sqrt((x_pos - posisix_awal)^2 + (y_pos - posisiy_awal)^2); % Jarak diagonal ke titik asal
        waktu_return = jarak / kecepatan; % Waktu tempuh
        t_segment = linspace(waktu_total, waktu_total + waktu_return, 100);
        x_segment = linspace(x_pos, posisix_awal, 100); % Gerakan dari x_pos ke posisix_awal
        y_segment = linspace(y_pos, posisiy_awal, 100); % Gerakan dari y_pos ke posisiy_awal
    end

    % Tambahkan lintasan kembali ke titik asal
    x_d = [x_d, x_segment];
    y_d = [y_d, y_segment];
    waktu_d = [waktu_d, t_segment];

    % Perbarui waktu total
    waktu_total = waktu_total + waktu_return;

    % Jika waktu simulasi masih tersisa, tambahkan posisi diam di titik asal
    if waktu_total < waktu_simulasi
        waktu_diam = linspace(waktu_total, waktu_simulasi, 100);
        x_d = [x_d, posisix_awal * ones(size(waktu_diam))]; % Tetap di x = posisix_awal
        y_d = [y_d, posisiy_awal * ones(size(waktu_diam))]; % Tetap di y = posisiy_awal
        waktu_d = [waktu_d, waktu_diam];
    end


    % Plot lintasan dengan animasi
    figure;
    for k = 1:length(x_d)
        % Plot lintasan dan posisi drone saat ini
        plot(x_d(1:k), y_d(1:k), 'r', 'LineWidth', 1.5);
        hold on;
        plot(x_d(k), y_d(k), 'bo', 'MarkerFaceColor', 'b'); % Posisi drone saat ini
        xlabel('Jarak x');
        ylabel('Jarak y');
        title('Lintasan Drone Zig-Zag');
        grid on;
        axis([min(x_d)-1, max(x_d)+1, min(y_d)-1, max(y_d)+1]);
        hold off;

        % Ambil data posisi terbaru (1x1)
        posisi_terbaru_x = x_d(k);
        posisi_terbaru_y = y_d(k);

        % Tampilkan posisi terbaru dan waktu ke Command Window
        fprintf('Waktu: %.2f detik | Posisi terbaru: x = %.2f, y = %.2f\n', waktu_d(k), posisi_terbaru_x, posisi_terbaru_y);

        % Pause untuk animasi
        % pause(0.01);

        % Hentikan animasi jika waktu simulasi habis
        if waktu_d(k) >= waktu_simulasi
            break;
        end
    end
end
