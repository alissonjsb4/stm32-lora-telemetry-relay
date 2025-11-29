import serial
import time
import threading
import sys

PORTA_SERIAL = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
BAUDRATE = 115200

# Variável de controle para as threads
stop_event = threading.Event()

def thread_leitura(ser: serial.Serial):
    """Função que lê continuamente da porta serial."""
    print("Thread de Leitura iniciada...")
    while not stop_event.is_set():
        try:
            # Tenta ler uma linha
            resposta = ser.readline().decode("utf-8", errors="ignore").strip()

            if resposta:
                print(f"[{time.strftime('%H:%M:%S')}] Recebido: {resposta}")
        except serial.SerialException as e:
            print(f"Erro de leitura serial: {e}")
            break # Sai do loop em caso de erro grave na porta
        except Exception as e:
            print(f"Erro inesperado na leitura: {e}")
            # Não use time.sleep() aqui, pois o timeout da porta já
            # controla a espera.

    print("Thread de Leitura finalizada.")

def thread_escrita(ser: serial.Serial, intervalo_segundos: int = 10):
    """Função que escreve periodicamente na porta serial."""
    comando = b"1\n"
    print("Thread de Escrita iniciada...")
    while not stop_event.is_set():
        try:
            ser.write(comando)
            print(f"[{time.strftime('%H:%M:%S')}] Enviado: 1")
        except serial.SerialException as e:
            print(f"Erro de escrita serial: {e}")
            break # Sai do loop em caso de erro grave na porta
        except Exception as e:
            print(f"Erro inesperado na escrita: {e}")
            break

        # Espera o intervalo, mas verifica o stop_event a cada segundo
        stop_event.wait(intervalo_segundos)

    print("Thread de Escrita finalizada.")

if __name__ == "__main__":
    try:
        # Inicialização da porta serial
        ser = serial.Serial(
            port=PORTA_SERIAL,
            baudrate=BAUDRATE,
            timeout=1  # Timeout para readline, importante para a thread de leitura
        )

        time.sleep(2)  # Aguarda a porta estabilizar

        print(f"Porta serial {PORTA_SERIAL} aberta com sucesso.")

        # Criação das threads
        leitura_thread = threading.Thread(target=thread_leitura, args=(ser,), daemon=True)
        escrita_thread = threading.Thread(target=thread_escrita, args=(ser, 10), daemon=True)

        # Início das threads
        leitura_thread.start()
        escrita_thread.start()

        print("Threads de leitura e escrita iniciadas. Pressione Ctrl+C para sair.")

        # O thread principal espera as threads rodarem, mas de forma que
        # possa reagir a um KeyboardInterrupt (Ctrl+C)
        while leitura_thread.is_alive() or escrita_thread.is_alive():
            leitura_thread.join(timeout=0.1)
            escrita_thread.join(timeout=0.1)


    except serial.SerialException as e:
        print(f"\nERRO: Não foi possível abrir a porta serial {PORTA_SERIAL}. Verifique se a porta está correta e se o dispositivo está conectado.")
        print(f"Detalhes do erro: {e}")
    except KeyboardInterrupt:
        print("\nInterrupção pelo usuário (Ctrl+C) detectada.")
    finally:
        # Sinaliza para as threads pararem
        stop_event.set()

        # Aguarda as threads terminarem (garantindo que não fiquem penduradas)
        if 'leitura_thread' in locals() and leitura_thread.is_alive():
            leitura_thread.join()
        if 'escrita_thread' in locals() and escrita_thread.is_alive():
            escrita_thread.join()

        # Fecha a porta serial se estiver aberta
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Porta serial fechada.")

    print("Script finalizado.")
