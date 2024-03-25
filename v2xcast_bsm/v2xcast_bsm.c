/* Copyright (C) Unex Technology Corporation - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

#define _GNU_SOURCE
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include "error_code_enum.h"
#include "error_code_user.h"
#include "poti_caster_service.h"
#include "us_caster_service.h"
#include "j2735_codec.h"
#include "frozen.h"

#define ERR_MSG_SZ 256
#define MALLOC(sz) malloc(sz)
#define CALLOC(n, sz) calloc((n), (sz))
#define FREE(ptr) free(ptr)


#define HOST_CONFIG_FILE_NAME "config_host.json"

#define EMERGENCY (1 << 0)

/* Thread type is using for application send and receive thread, the application thread type is an optional method depend on execute platform */
typedef enum app_thread_type {
    APP_THREAD_TX = 0,
    APP_THREAD_RX = 1
} app_thread_type_t;

typedef struct host_config {
    char id[4];  ///< id
    int tractionControlStatus;  ///< Traction control status, please refer to TractionControlStatus
    int antiLockBrakeStatus;  ///< Antilock brake status, please refer to AntiLockBrakeStatus
    int stabilityControlStatus;  ///< Stability control status, please refer to StabilityControlStatus
    int vehicleSafetyExtensions_option;  ///< Option for vehicle safety extensions, 0 for unused, 1 for in use
    int rightTurnSignalOn;  ///< Right turn signal status, 0 for signal off, 1 for signal on
    int leftTurnSignalOn;  ///< Left turn signal status, 0 for signal off, 1 for signal on
    int specialVehicleExtensions_option;  ///< Option for special vehicle extensions , 0 for unused, 1 for in use
    int lightBarInUse;  ///< Light bar status, 0 for unused, 1 for in use
    int sirenInUse;  ///< Siren status, 0 for unused, 1 for in use
} host_config_t;

static host_config_t host_config;
int sec_state;
bool app_running = true;
bool is_secured;

static void *receiver_handler(void *args);
static void *sender_handler(void *args);
void bsm_encode(uint8_t **tx_buf, int *tx_buf_len, poti_fix_data_t *fix_data);
void bsm_decode(uint8_t *rx_buf, int rx_buf_len);
void bsm_print(BasicSafetyMessage *bsm);
static int load_host_config(host_config_t *config);
static int32_t app_set_thread_name_and_priority(pthread_t thread, app_thread_type_t type, char *p_name, int32_t priority);
static void dump_rx_info(us_caster_rx_info_t *rx_info);
static void set_tx_info(us_caster_tx_info_t *tx_info, bool is_secured);

void bsm_encode(uint8_t **tx_buf, int *tx_buf_len, poti_fix_data_t *fix_data)
{
    MessageFrame msgf;
    BasicSafetyMessage *bsm;
    PartIIcontent *part2_sf_ext;
    PartIIcontent *part2_sp_ext;
    VehicleSafetyExtensions *sf_ext;
    SpecialVehicleExtensions *sp_ext;
    SelfDefineMsg *self_define_msg;
    char id[] = "abcd";
    static int msg_cnt = 0;
    int ret;

    /* Load host setting from configuration file */
    memset(&host_config, 0, sizeof(host_config));
    ret = load_host_config(&host_config);
    if (-1 == ret) {
        printf("Using fixed BSM data.\n");
    }

    memset(&msgf, 0, sizeof(msgf));
    /* all fields which should be allocated before using are allocated recursively */
    bsm = (BasicSafetyMessage *)j2735_msg_prealloc(BasicSafetyMessage_Id);

    /* The example only fills a part of BSMcoreData and provides a sample convert */
    /* The user could depend on the needed and sensor data from OBU to fill the rest of the element */
    bsm->coreData.msgCnt = (msg_cnt++) % 127;
    printf("message count %d\n", bsm->coreData.msgCnt);

    if (0 == ret) {
        /* Set id form host config file */
        asn1_ostr_clone_cstr(&(bsm->coreData.id), host_config.id, 4);
    }
    else {
        /* Set fixed id */
        asn1_ostr_clone_cstr(&(bsm->coreData.id), id, 4);
    }

    bsm->coreData.secMark = fix_data->time.utc.sec * 1000 + fix_data->time.utc.ms;
    bsm->coreData.Long = (int32_t)(fix_data->longitude * 10000000);
    bsm->coreData.lat = (int32_t)(fix_data->latitude * 10000000);
    bsm->coreData.speed = (int32_t)(fix_data->horizontal_speed * 50); /* Convert to 0.02 metre per second. */
    bsm->coreData.heading = (int32_t)(fix_data->course_over_ground * 80); /* Convert to 0.0125 degree from North. */
    printf("\n");
    printf("coreData.secMark: %d\n", bsm->coreData.secMark);
    printf("coreData.long: %lf\n", bsm->coreData.Long / 10000000.0);
    printf("coreData.lat: %lf\n", bsm->coreData.lat / 10000000.0);
    /* only set the bit of bitstring */
    asn1_bstr_set_bit(&(bsm->coreData.brakes.wheelBrakes), BrakeAppliedStatus_rightFront);

    if (0 == ret) {
        /* Set brakes form host config file */
        bsm->coreData.brakes.traction = host_config.tractionControlStatus;
        bsm->coreData.brakes.abs = host_config.antiLockBrakeStatus;
        bsm->coreData.brakes.scs = host_config.stabilityControlStatus;
    }
    else {
        /* Set fixed brakes */
        bsm->coreData.brakes.traction = TractionControlStatus_engaged;
        bsm->coreData.brakes.abs = AntiLockBrakeStatus_engaged;
        bsm->coreData.brakes.scs = StabilityControlStatus_engaged;
    }

    /* set the optional field to TRUE to include the data when encoding */
    bsm->partII_option = TRUE;
    /* the items in the list of partII are allocated with max. size */
    /* just set the number of items which are really filled */

    if (0 == ret) {
        bsm->partII.count = 0;
        /* Set partII form host config file */
        if (host_config.vehicleSafetyExtensions_option) {
            /* if the type of the item contains union field, choose the one you want and call prealloc assistant */
            part2_sf_ext = &(bsm->partII.tab[bsm->partII.count]);
            part2_sf_ext->partII_Id = VehicleSafetyExt;

            /* all fields which should be allocated before using are allocated recursively */
            j2735_dataframe_prealloc(PartIIcontent_DF, part2_sf_ext);

            /* set union to vehicle safety extensions */
            sf_ext = part2_sf_ext->u.safetyExt;
            sf_ext->events_option = TRUE;
            /* only set the bit of bitstring */
            asn1_bstr_set_bit(&(sf_ext->events), VehicleEventFlags_eventStopLineViolation);
            /* set the optional field to TRUE to include the data when encoding */
            sf_ext->pathPrediction_option = TRUE;
            sf_ext->pathPrediction.radiusOfCurve = 1000;
            sf_ext->pathPrediction.confidence = 100;
            /* set the optional field to TRUE to include the data when encoding */
            sf_ext->lights_option = TRUE;
            if (1 == host_config.leftTurnSignalOn) {
                asn1_bstr_set_bit(&(sf_ext->lights), ExteriorLights_leftTurnSignalOn);
            }
            if (1 == host_config.rightTurnSignalOn) {
                asn1_bstr_set_bit(&(sf_ext->lights), ExteriorLights_rightTurnSignalOn);
            }
            bsm->partII.count += 1;
        }
        if (host_config.specialVehicleExtensions_option) {
            /* if the type of the item contains union field, choose the one you want and call prealloc assistant */
            part2_sp_ext = &(bsm->partII.tab[bsm->partII.count]);
            part2_sp_ext->partII_Id = SpecialVehicleExt;
            /* all fields which should be allocated before using are allocated recursively */
            j2735_dataframe_prealloc(PartIIcontent_DF, part2_sp_ext);
            /* set union to special vehicle extensions */
            sp_ext = part2_sp_ext->u.specialExt;
            /* set the optional field to TRUE to include the data when encoding */
            sp_ext->vehicleAlerts_option = TRUE;
            /* set emergency detail content */
            sp_ext->vehicleAlerts.notUsed = 0;
            sp_ext->vehicleAlerts.sirenUse = host_config.sirenInUse;
            sp_ext->vehicleAlerts.lightsUse = host_config.lightBarInUse;
            sp_ext->vehicleAlerts.events_option = FALSE;
            sp_ext->vehicleAlerts.responseType_option = TRUE;
            sp_ext->vehicleAlerts.responseType = ResponseType_emergency;
            /* set the optional field to FALSE to not include the data when encoding */
            sp_ext->description_option = FALSE;
            sp_ext->trailers_option = FALSE;
            bsm->partII.count += 1;
        }
    }
    else {
        /* Set fixed partII */
        bsm->partII.count = 2;
        /* if the type of the item contains union field, choose the one you want and call prealloc assistant */
        part2_sf_ext = &(bsm->partII.tab[0]);
        part2_sf_ext->partII_Id = VehicleSafetyExt;
        part2_sp_ext = &(bsm->partII.tab[1]);
        part2_sp_ext->partII_Id = SpecialVehicleExt;
        /* all fields which should be allocated before using are allocated recursively */
        j2735_dataframe_prealloc(PartIIcontent_DF, part2_sf_ext);
        j2735_dataframe_prealloc(PartIIcontent_DF, part2_sp_ext);
        /* set union to vehicle safety extensions */
        sf_ext = part2_sf_ext->u.safetyExt;
        /* set the optional field to TRUE to include the data when encoding */
        sf_ext->events_option = TRUE;
        /* only set the bit of bitstring */
        asn1_bstr_set_bit(&(sf_ext->events), VehicleEventFlags_eventStopLineViolation);
        /* set the optional field to TRUE to include the data when encoding */
        sf_ext->pathPrediction_option = TRUE;
        sf_ext->pathPrediction.radiusOfCurve = 1000;
        sf_ext->pathPrediction.confidence = 100;
        /* set the optional field to TRUE to include the data when encoding */
        sf_ext->lights_option = TRUE;
        /* only set the bit of bitstring */
        asn1_bstr_set_bit(&(sf_ext->lights), ExteriorLights_leftTurnSignalOn);

        /* set union to special vehicle extensions */
        sp_ext = part2_sp_ext->u.specialExt;
        /* set the optional field to TRUE to include the data when encoding */
        sp_ext->vehicleAlerts_option = TRUE;
        /* set emergency detail content */
        sp_ext->vehicleAlerts.notUsed = 0;
        sp_ext->vehicleAlerts.sirenUse = SirenInUse_inUse;
        sp_ext->vehicleAlerts.lightsUse = LightbarInUse_inUse;
        sp_ext->vehicleAlerts.events_option = FALSE;
        sp_ext->vehicleAlerts.responseType_option = TRUE;
        sp_ext->vehicleAlerts.responseType = ResponseType_emergency;
        /* set the optional field to FALSE to not include the data when encoding */
        sp_ext->description_option = FALSE;
        sp_ext->trailers_option = FALSE;
    }

    /* does the encoding and providing the error msg if there is */
    msgf.messageId = BasicSafetyMessage_Id;
    // msgf.u.data = bsm;
    self_define_msg = 'Hello!';
    msgf.u.data = self_define_msg;

    *tx_buf_len = j2735_msg_encode(tx_buf, &msgf, NULL);
    if (*tx_buf_len <= 0) {
        printf("failed to encode the msg\n");
    }
    else {
        printf("encode successfully\n");
    }

    /* free the memory for encoding */
    for (int i = 0; (i < bsm->partII.count) && (i < PartIIcontentList_MAX_SIZE); i++) {
        j2735_dataframe_dealloc(PartIIcontent_DF, &(bsm->partII.tab[i]));
    }
    j2735_msg_dealloc(BasicSafetyMessage_Id, bsm);

    return;
}

void bsm_decode(uint8_t *rx_buf, int rx_buf_len)
{
    int ret;
    /* a pointer to containing decoded msg */
    MessageFrame *p_msgf;

    printf("BSM decoding data:\n");
    asn1_dump_buf(rx_buf, rx_buf_len);

    ret = j2735_msg_decode(&p_msgf, rx_buf, rx_buf_len, NULL);
    if (ret < 0) {
        /* handling the decoding error */
        printf("decode msg error\n");
    }
    else if ((ret > 0) && (p_msgf->messageId == BasicSafetyMessage_Id)) {
        bsm_print((BasicSafetyMessage *)(p_msgf->u.data));
        J2735_FREE_MSG_FRAME(p_msgf);
        printf("decode successfully\n");
    }

    return;
}

void bsm_print(BasicSafetyMessage *bsm)
{
    int i, fbs;

    printf("Decoded BSM\n");
    printf("  coreData.msgCnt: %u\n", bsm->coreData.msgCnt);
    printf("  coreData.id: ");
    /* the array-like type, such as string or byte array is described by buf and len */
    for (i = 0; i < bsm->coreData.id.len; i++) {
        printf("%hhx", bsm->coreData.id.buf[i]);
    }
    printf("\n");
    printf("  secMark: %d\n", bsm->coreData.secMark);
    printf("  transmission: %d\n", bsm->coreData.transmission);

    /* the data is included only when the optional field is TRUE */
    if (bsm->partII_option) {
        /* the number of items in "sequence of" could be gotten by field, "count"  */
        for (i = 0; i < bsm->partII.count; i++) {
            PartIIcontent *part2 = &(bsm->partII.tab[i]);
            printf("  partII[%d]:\n", i);
            printf("    partII_Id: %d\n", part2->partII_Id);
            switch (part2->partII_Id) {
                case VehicleSafetyExt:
                    printf("     case VehicleSafetyExt: ");
                    /* by the Find First Set bit function, you can get the flag */
                    fbs = asn1_bstr_ffs(&(part2->u.safetyExt->lights));
                    switch (fbs) {
                        case ExteriorLights_lowBeamHeadlightsOn:
                            printf("ExteriorLights_lowBeamHeadlightsOn\n");
                            break;
                        case ExteriorLights_highBeamHeadlightsOn:
                            printf("ExteriorLights_highBeamHeadlightsOn\n");
                            break;
                        case ExteriorLights_leftTurnSignalOn:
                            printf("ExteriorLights_leftTurnSignalOn\n");
                            break;
                        default:
                            printf("%d\n", fbs);
                    }
                    break;
                case SpecialVehicleExt:
                    break;
                case SupplementalVehicleExt:
                    break;
                default:
                    break;
            }
        }
    }

    return;
}

static void *receiver_handler(void *args)
{
    caster_handler_t rx_handler = INVALID_CASTER_HANDLER;
    caster_comm_config_t caster_config = *((caster_comm_config_t *)args);
    us_caster_rx_info_t rx_info;
    uint8_t rx_buf[US_CASTER_PKT_SIZE_MAX];
    size_t len;
    int ret;

    caster_config.caster_comm_mode = CASTER_MODE_RX;
    ret = us_caster_create(&rx_handler, &caster_config);
    if (!IS_SUCCESS(ret)) {
        printf("Cannot link to V2Xcast Service, V2Xcast Service create ret: [%d] %s!\n", ret, ERROR_MSG(ret));
        printf("Please confirm network connection by ping the Unex device then upload a V2Xcast config to create a V2Xcast Service.\n");
        return NULL;
    }

    while (app_running) {
        printf("-----------------------\n");
        ret = us_caster_rx(rx_handler, &rx_info, rx_buf, sizeof(rx_buf), &len);
        if (IS_SUCCESS(ret)) {
            printf("Received %zu bytes!\n", len);

            /* Display RX information */
            dump_rx_info(&rx_info);

            /* Display message content */
            bsm_decode(rx_buf, (int)len);
        }
        else {
            /* The user may still get the payload with len is non-zero. */
            /* It means the data has some problems, such as security check failure */
            /* Users can determine to trust the data or not by themself */
            if (len != 0) {
                printf("Received %zu bytes, but had some issues! err code is:%d, msg = %s\n", len, ret, ERROR_MSG(ret));
            }
            else {
                printf("Failed to receive data, err code is:%d, msg = %s\n", ret, ERROR_MSG(ret));
            }
        }
        fflush(stdout);
    }

    us_caster_release(rx_handler);

    pthread_exit(NULL);
}

static void *sender_handler(void *args)
{
    caster_handler_t tx_handler = INVALID_CASTER_HANDLER, poti_handler = INVALID_CASTER_HANDLER;
    caster_comm_config_t caster_config = *((caster_comm_config_t *)args);
    us_caster_tx_info_t tx_info = {0};
    poti_fix_data_t fix_data = {0};
    poti_gnss_info_t gnss_info = {0};
    uint8_t *tx_buf = NULL;
    int tx_buf_len = 0;
    int ret;
    timer_t tm;
    struct sigevent sev;
    struct itimerspec ts;
    struct itimerspec curr;

    caster_config.caster_comm_mode = CASTER_MODE_TX;
    ret = us_caster_create(&tx_handler, &caster_config);
    if (!IS_SUCCESS(ret)) {
        printf("Cannot link to V2Xcast Service, V2Xcast Service create ret: [%d] %s!\n", ret, ERROR_MSG(ret));
        printf("Please confirm network connection by ping the Unex device then upload a V2Xcast config to create a V2Xcast Service.\n");
        return NULL;
    }

    caster_config.caster_comm_mode = CASTER_MODE_POTI;
    ret = us_caster_create(&poti_handler, &caster_config);
    if (!IS_SUCCESS(ret)) {
        printf("Cannot link to V2Xcast Service, V2Xcast Service create ret: [%d] %s!\n", ret, ERROR_MSG(ret));
        printf("Please confirm network connection by ping the Unex device then upload a V2Xcast config to create a V2Xcast Service.\n");
        return NULL;
    }

    sev.sigev_notify = SIGEV_NONE;
    timer_create(CLOCK_MONOTONIC, &sev, &tm);

    while (app_running) {
        printf("-----------------------\n");
        ts.it_interval.tv_sec = 0;
        ts.it_interval.tv_nsec = 0;
        ts.it_value.tv_sec = 1;
        ts.it_value.tv_nsec = 0;
        timer_settime(tm, 0, &ts, NULL);

        /* Get GNSS fix data from caster service */
        ret = poti_caster_fix_data_get(poti_handler, &fix_data);

        if (ret != 0) {
            printf("Fail to receive GNSS fix data from caster service, %d, remain %ld ms\n", ret, curr.it_value.tv_nsec / 1000000);
            /* Waiting for POTI caster service startup */
            timer_gettime(tm, &curr);
            while (curr.it_value.tv_nsec > 0) {
                usleep(1000);
                timer_gettime(tm, &curr);
            }
            continue;
        }
        else if (fix_data.mode < POTI_FIX_MODE_2D) {
            printf("GNSS not fix, fix mode: %d\n, remain %ld ms", fix_data.mode, curr.it_value.tv_nsec / 1000000);

            /* Optional, APIs for users to get more GNSS information */
            ret = poti_caster_gnss_info_get(poti_handler, &gnss_info);
            if (IS_SUCCESS(ret)) {
                printf("GNSS antenna status:%d, time sync status: %d\n", gnss_info.antenna_status, gnss_info.time_sync_status);
            }

            /* Waiting for POTI caster service startup */
            timer_gettime(tm, &curr);
            while (curr.it_value.tv_nsec > 0) {
                usleep(1000);
                timer_gettime(tm, &curr);
            }
            continue;
        }

        /* Optional, NAN value check for GNSS data */
        if ((isnan(fix_data.latitude) == 1) || (isnan(fix_data.longitude) == 1) || (isnan(fix_data.altitude) == 1) || (isnan(fix_data.horizontal_speed) == 1) || (isnan(fix_data.course_over_ground) == 1)) {
            printf("GNSS fix data has NAN value, latitude: %f, longitude : %f, altitude : %f, horizontal speed : %f, course over ground : %f\n", fix_data.latitude, fix_data.longitude, fix_data.altitude, fix_data.horizontal_speed, fix_data.course_over_ground);
            timer_gettime(tm, &curr);
            while (curr.it_value.tv_nsec > 0) {
                usleep(1000);
                timer_gettime(tm, &curr);
            }

            /* Waiting for POTI caster service startup */
            timer_gettime(tm, &curr);
            while (curr.it_value.tv_nsec > 0) {
                usleep(1000);
                timer_gettime(tm, &curr);
            }
            continue;
        }

        timer_gettime(tm, &curr);
        if (curr.it_value.tv_nsec < 900 * 1000 * 1000) {
            printf("remain %ld ms, poti_caster_fix_data_get exceed limitation(100ms)\n", curr.it_value.tv_nsec / 1000000);

            /* Waiting for timeout */
            timer_gettime(tm, &curr);
            while (curr.it_value.tv_nsec > 0) {
                usleep(1000);
                timer_gettime(tm, &curr);
            }
        }

        bsm_encode(&tx_buf, &tx_buf_len, &fix_data);

        printf("BSM encoded data:\n");
        asn1_dump_buf(tx_buf, tx_buf_len);

        set_tx_info(&tx_info, is_secured);

        ret = us_caster_tx(tx_handler, &tx_info, tx_buf, (size_t)tx_buf_len);

        if (IS_SUCCESS(ret)) {
            printf("Transmitted %d bytes!\n", tx_buf_len);
            printf("Transmitted BSM\n");
        }
        else {
            printf("Failed to transmit data, err code is:%d, msg = %s\n", ret, ERROR_MSG(ret));
            printf("Failed to transmit BSM\n");
        }
        fflush(stdout);
        j2735_buf_free(tx_buf);

        /* Waiting for timeout */
        timer_gettime(tm, &curr);
        while (curr.it_value.tv_nsec > 0) {
            usleep(1000);
            timer_gettime(tm, &curr);
        }
    }

    timer_delete(tm);
    us_caster_release(tx_handler);
    printf("tx_caster_release done\n");
    us_caster_release(poti_handler);
    printf("us_caster_release done\n");
    pthread_exit(NULL);
}

void app_signal_handler(int sig_num)
{
    if (sig_num == SIGINT) {
        printf("SIGINT signal!\n");
    }
    if (sig_num == SIGTERM) {
        printf("SIGTERM signal!\n");
    }
    app_running = false;
}

char app_sigaltstack[SIGSTKSZ];
int app_setup_signals(void)
{
    stack_t sigstack;
    struct sigaction sa;
    int ret = -1;

    sigstack.ss_sp = app_sigaltstack;
    sigstack.ss_size = SIGSTKSZ;
    sigstack.ss_flags = 0;
    if (sigaltstack(&sigstack, NULL) == -1) {
        perror("signalstack()");
        goto END;
    }

    sa.sa_handler = app_signal_handler;
    sa.sa_flags = SA_ONSTACK;
    if (sigaction(SIGINT, &sa, 0) != 0) {
        perror("sigaction()");
        goto END;
    }
    if (sigaction(SIGTERM, &sa, 0) != 0) {
        perror("sigaction()");
        goto END;
    }

    ret = 0;
END:
    return ret;
}

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    caster_comm_config_t config;
    pthread_t thread1;
    pthread_t thread2;
    int ret;
    J2735Config cfg;
    caster_thread_info_t caster_thread_info;

    setbuf(stdout, NULL);

    if (argc != 4) {
        printf("v2xcast_bsm <IP_of_V2Xcast_service> <is_send> <is_sec 0:none 1:signed 2:unsecured>\n");
        return -1;
    }

    config.ip = argv[1];
    config.caster_id = 0;

    is_secured = atoi(argv[3]);

    ret = app_setup_signals();
    if (!IS_SUCCESS(ret)) {
        printf("Fail to app_setup_signals\n");
        return -1;
    }

    ret = j2735_init(&cfg);
    if (!IS_SUCCESS(ret)) {
        printf("Fail to init J2735\n");
        return -1;
    }

    if (atoi(argv[2]) == 0) { /* receiving thread */
        us_caster_init();

        if (pthread_create(&thread1, NULL, receiver_handler, (void *)&config)) {
            perror("could not create thread for receiver_handler");
            return -1;
        }
        /* If the example is run in Unex device, please using the below functions to set tx and rx message threads name and priority */
        /* If the example is run on other platforms, it is optional to set tx and rx message threads name and priority */
        us_caster_thread_info_get(&caster_thread_info);
        ret = app_set_thread_name_and_priority(thread1, APP_THREAD_RX, caster_thread_info.rx_thread_name, caster_thread_info.rx_thread_priority_low);

        pause();

        us_caster_deinit();
        printf("us_caster_deinit done\n");
        pthread_join(thread1, NULL);
        printf("pthread_join done\n");
    }
    else { /* sending thread */
        us_caster_init();

        if (pthread_create(&thread2, NULL, sender_handler, (void *)&config)) {
            perror("could not create thread for sender_handler");
            return -1;
        }

        /* If the example is run in Unex device, please using the below functions to set tx and rx message threads name and priority */
        /* If the example is run on other platforms, it is optional to set tx and rx message threads name and priority */
        us_caster_thread_info_get(&caster_thread_info);
        ret = app_set_thread_name_and_priority(thread2, APP_THREAD_TX, caster_thread_info.tx_thread_name, caster_thread_info.tx_thread_priority_low);

        pause();
        us_caster_deinit();
        printf("us_caster_deinit done\n");
        pthread_join(thread2, NULL);
        printf("pthread_join done\n");
    }
    return 0;
}

/**
 *
 * @fn load_host_config(host_config_t *config)
 * @brief   load host config from file
 * @param   [out] config configuration read form file
 *
 * @return  [int]   Function executing result.
 * @retval  [0]     Success.
 * @retval  [-1]    Fail.
 */
static int load_host_config(host_config_t *config)
{
    char *content = json_fread(HOST_CONFIG_FILE_NAME);
    if (content == NULL) {
        /* File read failed, create the default config file */
        printf("The host config file not exist, create the default config file!\n");
        json_fprintf(HOST_CONFIG_FILE_NAME,
                     "{ \
                        SendBSM : [   \
                        { \
                            id : abcd, \
                            tractionControlStatus : 3, \
                            antiLockBrakeStatus : 3, \
                            stabilityControlStatus : 3, \
                            vehicleSafetyExtensions : \
                            { \
                                option : 1, \
                                rightTurnSignalOn : 1, \
                                leftTurnSignalOn : 0, \
                            }, \
                            specialVehicleExtensions :  \
                            { \
                                option : 1, \
                                lightBarInUse : 1, \
                                sirenInUse : 2, \
                            } \
                        } \
                        ] \
                    }");
        json_prettify_file(HOST_CONFIG_FILE_NAME);  // Optional
        content = json_fread(HOST_CONFIG_FILE_NAME);
    }

    if (content != NULL) {
        /* Extract setting form JSON format */
        struct json_token t_root;
        int i, len = strlen(content);

        for (i = 0; json_scanf_array_elem(content, len, ".SendBSM", i, &t_root) > 0; i++) {
            char *id = NULL;
            int id_len = 0;

            printf("Index %d, token %.*s\n", i, t_root.len, t_root.ptr);
            json_scanf(t_root.ptr, t_root.len, "{id: %Q}", &id);

            id_len = strlen(id);
            for (int i = 0; ((i < id_len) && (i < 4)); i++) {
                config->id[i] = id[i];
            }
            json_scanf(t_root.ptr, t_root.len, "{tractionControlStatus: %d, antiLockBrakeStatus: %d, stabilityControlStatus: %d}", &(config->tractionControlStatus), &(config->antiLockBrakeStatus), &(config->stabilityControlStatus));
            json_scanf(t_root.ptr, t_root.len, "{vehicleSafetyExtensions: {option: %d, rightTurnSignalOn: %d, leftTurnSignalOn: %d}}", &(config->vehicleSafetyExtensions_option), &(config->rightTurnSignalOn), &(config->leftTurnSignalOn));
            json_scanf(t_root.ptr, t_root.len, "{specialVehicleExtensions: {option: %d, lightBarInUse: %d, sirenInUse: %d}}", &(config->specialVehicleExtensions_option), &(config->lightBarInUse), &(config->sirenInUse));
            printf("BSM id: %c%c%c%c, tractionControlStatus: %d, antiLockBrakeStatus: %d, stabilityControlStatus: %d, vehicleSafetyExtensions_option: %d, rightTurnSignalOn: %d, leftTurnSignalOn: %d, specialVehicleExtensions_option: %d,lightBarInUse: %d, sirenInUse: %d\n", config->id[0], config->id[1], config->id[2], config->id[3], config->tractionControlStatus, config->antiLockBrakeStatus, config->stabilityControlStatus, config->vehicleSafetyExtensions_option, config->rightTurnSignalOn, config->leftTurnSignalOn, config->specialVehicleExtensions_option, config->lightBarInUse, config->sirenInUse);
            if (id != NULL) {
                free(id);
            }
        }
        free(content);
        return 0;
    }
    else {
        printf("Load host config file failed!\n");
        return -1;
    }

    return 0;
}

static void dump_rx_info(us_caster_rx_info_t *rx_info)
{
    struct tm *timeinfo;
    char buffer[80];
    time_t t;

    printf("Dump rx info:\n");
    t = rx_info->timestamp.tv_sec;
    timeinfo = localtime(&t);
    strftime(buffer, 80, "%Y%m%d%H%M%S", timeinfo);
    printf("  timestamp:%s\n", buffer);
    printf("  channel used:%hd\n", rx_info->channel_used);
    printf("  rssi:%hd\n", rx_info->rssi);
    if (rx_info->channel_number_is_present) {
        printf("  channel number:%d\n", rx_info->channel_number);
    }
    if (rx_info->tx_power_is_present) {
        printf("  tx power:%d\n", rx_info->tx_power);
    }
    if (rx_info->data_rate_is_present) {
        printf("  data rate:%d\n", rx_info->data_rate);
    }
    if (is_secured) {
        printf("  decap status:%d\n", rx_info->security.status);
        switch (rx_info->security.status) {
            case US_SEC_DECAP_VERIFIED_PKT:
                printf("\tSecurity status: this packet is verified\n");
                printf("\tssp_len = %hu\n", rx_info->security.ssp_len);
                for (uint8_t i = 0; i < rx_info->security.ssp_len; i++) {
                    printf("\tssp[%hu]=%hu\n", i, rx_info->security.ssp[i]);
                }
                break;
            case US_SEC_DECAP_UNVERIFIABLE_PKT:
                printf("\tSecurity status:  this packet is untrustworthy\n");
                break;
            case US_SEC_DECAP_INVALID_FMT:
                printf("\tSecurity status: decapsulation error (%d), the payload content is invalid\n", rx_info->security.status);
                break;
            default:
                printf("\tSecurity status: other (%d)\n", rx_info->security.status);
                break;
        }
    }
    return;
}

static void set_tx_info(us_caster_tx_info_t *tx_info, bool is_secured)
{
    /* set data rate*/
    tx_info->data_rate_is_present = true;
    tx_info->data_rate = 12; /* 12 (500kbps) = 6 (Mbps) */

    if (is_secured) {
        /* set security */
        tx_info->ssp_is_present = true;

        /*
        * Assign BSM service specific permissions according to the actual content in payload.
        * This SSP design is for Unex BSM example demonstrating only.
        * For the field usage, please refer to the SAE J2945/5 standard to modify the SSP demo design in below.
        * ++----------------------+----------------------+-----------------------+
        * +| Entity Activity      | Entity Activities    | SSP Activity Bit      |
        * +| Group                |                      |                       |
        * ++======================+======================+=======================+
        * +| Special Vehicle      | Emergency vehicle    | Bit 1, (8th bit in    |
        * +| Extensions           | active               | the SSP)              |
        * ++----------------------+----------------------+-----------------------+
        *
        * Bits 1 through 8: Version number. Set to one for this version (00000001).
        * Bits 9 through 16: SSP activity bits on the table above.
        */

        /* SSP Version control */
        tx_info->ssp[0] = 0x1;
        /* Service-specific parameter */
        tx_info->ssp[1] = EMERGENCY; /* Emergency vehicle active for Unex example demo usage */
        tx_info->ssp_len = 1;
    }

    return;
}

static int32_t app_set_thread_name_and_priority(pthread_t thread, app_thread_type_t type, char *p_name, int32_t priority)
{
    int32_t result;
    caster_thread_info_t limited_thread_config;

#ifdef __SET_PRIORITY__
    struct sched_param param;
#endif  // __SET_PRIORITY__
    if (p_name == NULL) {
        return -1;
    }

    /* Check thread priority is in the limited range */
    us_caster_thread_info_get(&limited_thread_config);

    if (APP_THREAD_TX == type) {
        /* Check the limited range for tx thread priority */
        if ((priority < limited_thread_config.tx_thread_priority_low) || (priority > limited_thread_config.tx_thread_priority_high)) {
            /* Thread priority is out of range */
            printf("The tx thread priority is out of range (%d-%d): %d \n", limited_thread_config.tx_thread_priority_low, limited_thread_config.tx_thread_priority_high, priority);
            return -1;
        }
    }
    else if (APP_THREAD_RX == type) {
        /* Check the limited range for rx thread priority */
        if ((priority < limited_thread_config.rx_thread_priority_low) || (priority > limited_thread_config.rx_thread_priority_high)) {
            /* Thread priority is out of range */
            printf("The rx thread priority is out of range (%d-%d): %d \n", limited_thread_config.rx_thread_priority_low, limited_thread_config.rx_thread_priority_high, priority);
            return -1;
        }
    }
    else {
        /* Target thread type is unknown */
        printf("The thread type is unknown: %d \n", type);
        return -1;
    }

    result = pthread_setname_np(thread, p_name);
    if (result != 0) {
        printf("Can't set thread name: %d (%s)\n", result, strerror(result));
        return -1;
    }

#ifdef __SET_PRIORITY__
    param.sched_priority = priority;
    result = pthread_setschedparam(thread, SCHED_FIFO, &param);
    if (result != 0) {
        printf("Can't set thread priority: %d (%s)\n", result, strerror(result));
        return -1;
    }
#endif  // __SET_PRIORITY__
    return 0;
}
