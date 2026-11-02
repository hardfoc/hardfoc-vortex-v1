/* Host check of the generated object dictionary against CANopenNode v4.
 *
 * 1. Every entry is found through the OD interface with the expected
 *    attribute, length and default bytes (od_expect.h, written by pytest
 *    from the YAML model).
 * 2. The stack accepts the dictionary: CO_new, CO_CANinit, CO_CANopenInit
 *    and CO_CANopenInitPDO succeed with node-ID 5 (counts, entry pointers,
 *    SDO server 2 parameters and every default PDO mapping are validated by
 *    the stack itself).
 */
#include <stdio.h>
#include <string.h>

#include "CANopen.h"
#include "OD.h"
#include "od_expect.h"

static int failures = 0;
#define EXPECT(c, ...)                         \
    do {                                       \
        if (!(c)) {                            \
            ++failures;                        \
            printf("FAIL %s: ", #c);           \
            printf(__VA_ARGS__);               \
            printf("\n");                      \
        }                                      \
    } while (0)

static void check_entries(void) {
    size_t i;
    for (i = 0; i < sizeof(kExpect) / sizeof(kExpect[0]); ++i) {
        const od_expect_t* x = &kExpect[i];
        OD_entry_t* entry = OD_find(OD, x->index);
        OD_IO_t io;
        EXPECT(entry != NULL, "0x%04X missing", x->index);
        if (entry == NULL) {
            continue;
        }
        EXPECT(OD_getSub(entry, x->sub, &io, false) == ODR_OK, "0x%04X:%02X not found", x->index, x->sub);
        EXPECT(io.stream.attribute == x->attribute, "0x%04X:%02X attribute 0x%02X, want 0x%02X", x->index, x->sub,
               io.stream.attribute, x->attribute);
        EXPECT(io.stream.dataLength == x->length, "0x%04X:%02X length %u, want %u", x->index, x->sub,
               (unsigned)io.stream.dataLength, (unsigned)x->length);
        EXPECT((io.stream.dataOrig != NULL) == (x->has_data != 0), "0x%04X:%02X storage", x->index, x->sub);
        if (x->has_data && x->length <= sizeof(x->bytes)) {
            uint8_t buf[64];
            OD_size_t n = 0;
            memset(buf, 0, sizeof(buf));
            EXPECT(io.read(&io.stream, buf, x->length, &n) == ODR_OK, "0x%04X:%02X read", x->index, x->sub);
            EXPECT(memcmp(buf, x->bytes, x->compare_len) == 0, "0x%04X:%02X default bytes differ", x->index,
                   x->sub);
        }
    }
    printf("%u entries checked\n", (unsigned)(sizeof(kExpect) / sizeof(kExpect[0])));
}

static void check_stack_init(void) {
    uint32_t heap = 0;
    uint32_t errInfo = 0;
    CO_config_t* cfg = NULL;
#ifdef CO_MULTIPLE_OD
    CO_config_t co_config;
    memset(&co_config, 0, sizeof(co_config));
    OD_INIT_CONFIG(co_config);
    cfg = &co_config;
#endif
    CO_t* co = CO_new(cfg, &heap);
    EXPECT(co != NULL, "CO_new");
    if (co == NULL) {
        return;
    }
    EXPECT(CO_CANinit(co, NULL, 500) == CO_ERROR_NO, "CO_CANinit");
    CO_ReturnError_t err = CO_CANopenInit(co, NULL, NULL, OD, NULL,
                                          CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION,
                                          500, 1000, 500, true, 5, &errInfo);
    EXPECT(err == CO_ERROR_NO, "CO_CANopenInit err %d info 0x%08X", (int)err, (unsigned)errInfo);
    err = CO_CANopenInitPDO(co, co->em, OD, 5, &errInfo);
    EXPECT(err == CO_ERROR_NO, "CO_CANopenInitPDO err %d info 0x%08X", (int)err, (unsigned)errInfo);
    if (OD_find(OD, 0x1200) != NULL) {
        uint32_t cob = 0;
        EXPECT(OD_get_u32(OD_find(OD, 0x1200), 1, &cob, true) == ODR_OK && cob == 0x605U,
               "SDO server 1 COB-ID 0x%X", (unsigned)cob);
    }
    /* every default PDO mapping was accepted with the expected length and entry count */
    size_t i;
    for (i = 0; i < sizeof(kPdoExpect) / sizeof(kPdoExpect[0]); ++i) {
        const pdo_expect_t* p = &kPdoExpect[i];
        if (p->n == 0U) {
            continue; /* sentinel of an OD without PDOs */
        }
#if OD_CNT_RPDO > 0 && OD_CNT_TPDO > 0
        const CO_PDO_common_t* c = p->tx ? &co->TPDO[p->n - 1].PDO_common : &co->RPDO[p->n - 1].PDO_common;
        EXPECT(c->mappedObjectsCount == p->count, "%cPDO%u mapped %u, want %u", p->tx ? 'T' : 'R', p->n,
               c->mappedObjectsCount, p->count);
        EXPECT(c->dataLength == p->bytes, "%cPDO%u length %u, want %u", p->tx ? 'T' : 'R', p->n,
               (unsigned)c->dataLength, p->bytes);
        EXPECT((c->valid != 0) == (p->valid != 0), "%cPDO%u valid %d", p->tx ? 'T' : 'R', p->n, (int)c->valid);
#endif
    }
    printf("%u PDOs checked\n", (unsigned)(sizeof(kPdoExpect) / sizeof(kPdoExpect[0])));
    printf("stack init ok, %u bytes of CANopen objects\n", (unsigned)heap);
    CO_delete(co);
}

int main(void) {
    check_entries();
#ifndef ENTRIES_ONLY
    check_stack_init();
#else
    (void)check_stack_init; /* the stack still compiles and links against this OD */
#endif
    printf("%d failures\n", failures);
    return failures == 0 ? 0 : 1;
}
