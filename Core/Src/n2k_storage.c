/**
 * n2k_storage.c
 *
 * See n2k_storage.h for layout.  Two values share the same 1 KB flash page
 * (word[1] = source address, word[2] = instance reserve).  Because the
 * whole page must be erased before any word can be rewritten, every Save
 * function reads the current state of both values first, updates the
 * relevant one, then erases and writes back all three words together.
 */
#include "n2k_storage.h"
#include "stm32f1xx_hal.h"

#define N2K_STORAGE_ADDR   (0x0801FC00UL)   /* last 1 KB page, 127 KB limit in LD */
#define N2K_STORAGE_PAGE   (0x0801FC00UL)
#define N2K_STORAGE_MAGIC  (0x4E324B53UL)   /* 'N','2','K','S' */

#define WORD_ADDR(n)  (N2K_STORAGE_ADDR + (uint32_t)((n) * 4u))

/* Pack / unpack helpers ---------------------------------------------------- */

static inline uint32_t pack_byte(uint8_t a)
{
    return ((uint32_t)a) | ((uint32_t)((uint8_t)~a) << 8);
}

static bool unpack_byte(uint32_t w, uint8_t *out)
{
    uint8_t a  = (uint8_t)(w & 0xFFu);
    uint8_t na = (uint8_t)((w >> 8) & 0xFFu);
    if ((uint8_t)~a != na) { return false; }
    *out = a;
    return true;
}

static inline uint32_t read_word(uint8_t index)
{
    return *(const volatile uint32_t *)WORD_ADDR(index);
}

/* Public API --------------------------------------------------------------- */

bool N2kStorage_LoadSource(uint8_t *out_addr)
{
    if (out_addr == NULL) { return false; }
    if (read_word(0) != N2K_STORAGE_MAGIC) { return false; }
    return unpack_byte(read_word(1), out_addr);
}

bool N2kStorage_LoadInstance(uint8_t *out_instance)
{
    if (out_instance == NULL) { return false; }
    if (read_word(0) != N2K_STORAGE_MAGIC) { return false; }
    return unpack_byte(read_word(2), out_instance);
}

/** Erase the page and write all three words (magic + source + instance). */
static bool flash_write_all(uint8_t src_addr, uint8_t instance)
{
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef er = {0};
    er.TypeErase   = FLASH_TYPEERASE_PAGES;
    er.Banks       = FLASH_BANK_1;
    er.PageAddress = N2K_STORAGE_PAGE;
    er.NbPages     = 1u;

    uint32_t page_err = 0u;
    if (HAL_FLASHEx_Erase(&er, &page_err) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return false;
    }

    HAL_StatusTypeDef s0 = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, WORD_ADDR(0), N2K_STORAGE_MAGIC);
    HAL_StatusTypeDef s1 = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, WORD_ADDR(1), pack_byte(src_addr));
    HAL_StatusTypeDef s2 = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, WORD_ADDR(2), pack_byte(instance));

    HAL_FLASH_Lock();
    return (s0 == HAL_OK) && (s1 == HAL_OK) && (s2 == HAL_OK);
}

bool N2kStorage_SaveSource(uint8_t addr)
{
    /* Read current values; use defaults if not yet stored */
    uint8_t cur_addr     = 0u;
    uint8_t cur_instance = 0u;
    bool    addr_ok      = N2kStorage_LoadSource(&cur_addr);
    N2kStorage_LoadInstance(&cur_instance);

    /* Skip write when the source address already matches */
    if (addr_ok && cur_addr == addr) { return true; }

    return flash_write_all(addr, cur_instance);
}

bool N2kStorage_SaveInstance(uint8_t instance)
{
    /* Read current values; use defaults if not yet stored */
    uint8_t cur_addr     = 0u;
    uint8_t cur_instance = 0u;
    N2kStorage_LoadSource(&cur_addr);
    bool    inst_ok      = N2kStorage_LoadInstance(&cur_instance);

    /* Skip write when the instance already matches */
    if (inst_ok && cur_instance == instance) { return true; }

    return flash_write_all(cur_addr, instance);
}
