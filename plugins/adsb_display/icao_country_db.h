#pragma once

// ICAO 24-bit address → country lookup database
// Source: ICAO Annex 10 Vol III, Doc 8585 (address block allocations)
//
// Usage:
//   auto* entry = icao_lookup_country(0xA12345);
//   if (entry) printf("%s (%s)\n", entry->name, entry->code);

#include <algorithm>
#include <cstdint>

struct IcaoCountryEntry
{
    uint32_t start;       // range start (inclusive)
    uint32_t end;         // range end (inclusive)
    char code[3];         // ISO 3166-1 alpha-2, null-terminated
    const char* name;
};

// sorted by start address — binary search safe
static constexpr IcaoCountryEntry k_icao_country_db[] = {
    {0x004000, 0x0043FF, "ZW", "Zimbabwe"},
    {0x006000, 0x006FFF, "MZ", "Mozambique"},
    {0x008000, 0x00FFFF, "ZA", "South Africa"},
    {0x010000, 0x017FFF, "EG", "Egypt"},
    {0x018000, 0x01FFFF, "LY", "Libya"},
    {0x020000, 0x027FFF, "MA", "Morocco"},
    {0x028000, 0x02FFFF, "TN", "Tunisia"},
    {0x030000, 0x0303FF, "BW", "Botswana"},
    {0x032000, 0x032FFF, "BI", "Burundi"},
    {0x034000, 0x034FFF, "CM", "Cameroon"},
    {0x035000, 0x0353FF, "KM", "Comoros"},
    {0x036000, 0x036FFF, "CG", "Congo"},
    {0x038000, 0x038FFF, "CI", "Ivory Coast"},
    {0x03E000, 0x03EFFF, "GA", "Gabon"},
    {0x040000, 0x040FFF, "ET", "Ethiopia"},
    {0x042000, 0x042FFF, "GQ", "Equatorial Guinea"},
    {0x044000, 0x044FFF, "GH", "Ghana"},
    {0x046000, 0x046FFF, "GN", "Guinea"},
    {0x048000, 0x0483FF, "GW", "Guinea-Bissau"},
    {0x04A000, 0x04A3FF, "LS", "Lesotho"},
    {0x04C000, 0x04CFFF, "KE", "Kenya"},
    {0x050000, 0x050FFF, "LR", "Liberia"},
    {0x054000, 0x054FFF, "MG", "Madagascar"},
    {0x058000, 0x058FFF, "MW", "Malawi"},
    {0x05A000, 0x05A3FF, "MV", "Maldives"},
    {0x05C000, 0x05CFFF, "ML", "Mali"},
    {0x05E000, 0x05E3FF, "MR", "Mauritania"},
    {0x060000, 0x0603FF, "MU", "Mauritius"},
    {0x062000, 0x062FFF, "NE", "Niger"},
    {0x064000, 0x064FFF, "NG", "Nigeria"},
    {0x068000, 0x068FFF, "UG", "Uganda"},
    {0x06A000, 0x06A3FF, "QA", "Qatar"},
    {0x06C000, 0x06CFFF, "CF", "Central African Republic"},
    {0x06E000, 0x06EFFF, "RW", "Rwanda"},
    {0x070000, 0x070FFF, "SN", "Senegal"},
    {0x074000, 0x0743FF, "SC", "Seychelles"},
    {0x076000, 0x0763FF, "SL", "Sierra Leone"},
    {0x078000, 0x078FFF, "SO", "Somalia"},
    {0x07A000, 0x07A3FF, "SZ", "Eswatini"},
    {0x07C000, 0x07CFFF, "SD", "Sudan"},
    {0x080000, 0x080FFF, "TZ", "Tanzania"},
    {0x084000, 0x084FFF, "TD", "Chad"},
    {0x088000, 0x088FFF, "TG", "Togo"},
    {0x08A000, 0x08AFFF, "ZM", "Zambia"},
    {0x08C000, 0x08CFFF, "CD", "DR Congo"},
    {0x090000, 0x090FFF, "AO", "Angola"},
    {0x092000, 0x0923FF, "BJ", "Benin"},
    {0x094000, 0x0943FF, "CV", "Cape Verde"},
    {0x096000, 0x0963FF, "DJ", "Djibouti"},
    {0x098000, 0x0983FF, "GM", "Gambia"},
    {0x09A000, 0x09AFFF, "BF", "Burkina Faso"},
    {0x09C000, 0x09CFFF, "ST", "Sao Tome & Principe"},
    {0x0A0000, 0x0A7FFF, "DZ", "Algeria"},
    {0x0A8000, 0x0A8FFF, "BS", "Bahamas"},
    {0x0AA000, 0x0AA3FF, "BB", "Barbados"},
    {0x0AB000, 0x0AB3FF, "BZ", "Belize"},
    {0x0AC000, 0x0ACFFF, "CO", "Colombia"},
    {0x0AE000, 0x0AEFFF, "CR", "Costa Rica"},
    {0x0B0000, 0x0B0FFF, "CU", "Cuba"},
    {0x0B2000, 0x0B2FFF, "SV", "El Salvador"},
    {0x0B4000, 0x0B4FFF, "GT", "Guatemala"},
    {0x0B6000, 0x0B6FFF, "GY", "Guyana"},
    {0x0B8000, 0x0B8FFF, "HT", "Haiti"},
    {0x0BA000, 0x0BAFFF, "HN", "Honduras"},
    {0x0BC000, 0x0BC3FF, "VC", "St Vincent & Grenadines"},
    {0x0BE000, 0x0BEFFF, "JM", "Jamaica"},
    {0x0C0000, 0x0C0FFF, "NI", "Nicaragua"},
    {0x0C2000, 0x0C2FFF, "PA", "Panama"},
    {0x0C4000, 0x0C4FFF, "DO", "Dominican Republic"},
    {0x0C6000, 0x0C6FFF, "TT", "Trinidad and Tobago"},
    {0x0C8000, 0x0C8FFF, "SR", "Suriname"},
    {0x0CA000, 0x0CA3FF, "AG", "Antigua and Barbuda"},
    {0x0CC000, 0x0CC3FF, "GD", "Grenada"},
    {0x0D0000, 0x0D7FFF, "MX", "Mexico"},
    {0x0D8000, 0x0DFFFF, "VE", "Venezuela"},
    {0x0E0000, 0x0E3FFF, "PE", "Peru"},
    {0x0E4000, 0x0E7FFF, "AR", "Argentina"},
    {0x0E8000, 0x0EBFFF, "BR", "Brazil"},
    {0x0EC000, 0x0ECFFF, "CL", "Chile"},
    {0x0EE000, 0x0EEFFF, "EC", "Ecuador"},
    {0x0F0000, 0x0F0FFF, "PY", "Paraguay"},
    {0x0F2000, 0x0F2FFF, "UY", "Uruguay"},
    {0x0F4000, 0x0F4FFF, "BO", "Bolivia"},
    {0x100000, 0x1FFFFF, "IT", "Italy"},
    {0x200000, 0x27FFFF, "ES", "Spain"},
    {0x280000, 0x2FFFFF, "FR", "France"},
    {0x300000, 0x33FFFF, "DE", "Germany"},
    {0x340000, 0x37FFFF, "GB", "United Kingdom"},
    {0x380000, 0x3BFFFF, "HU", "Hungary"},
    {0x383000, 0x383FFF, "HU", "Hungary"},
    {0x3C0000, 0x3FFFFF, "DE", "Germany"},
    {0x400000, 0x43FFFF, "GB", "United Kingdom"},
    {0x440000, 0x447FFF, "AT", "Austria"},
    {0x448000, 0x44FFFF, "BE", "Belgium"},
    {0x450000, 0x457FFF, "BG", "Bulgaria"},
    {0x458000, 0x45FFFF, "DK", "Denmark"},
    {0x460000, 0x467FFF, "FI", "Finland"},
    {0x468000, 0x46FFFF, "GR", "Greece"},
    {0x470000, 0x477FFF, "HU", "Hungary"},
    {0x478000, 0x47FFFF, "NO", "Norway"},
    {0x480000, 0x487FFF, "NL", "Netherlands"},
    {0x488000, 0x48FFFF, "PL", "Poland"},
    {0x490000, 0x497FFF, "PT", "Portugal"},
    {0x498000, 0x49FFFF, "CZ", "Czech Republic"},
    {0x4A0000, 0x4A7FFF, "RO", "Romania"},
    {0x4A8000, 0x4AFFFF, "SE", "Sweden"},
    {0x4B0000, 0x4B7FFF, "CH", "Switzerland"},
    {0x4B8000, 0x4BFFFF, "TR", "Turkey"},
    {0x4C0000, 0x4C7FFF, "RS", "Serbia"},
    {0x4C8000, 0x4C83FF, "CY", "Cyprus"},
    {0x4CA000, 0x4CAFFF, "IE", "Ireland"},
    {0x4CC000, 0x4CCFFF, "IS", "Iceland"},
    {0x4D0000, 0x4D03FF, "LU", "Luxembourg"},
    {0x4D2000, 0x4D23FF, "MT", "Malta"},
    {0x4D4000, 0x4D43FF, "MC", "Monaco"},
    {0x500000, 0x5003FF, "SM", "San Marino"},
    {0x501000, 0x5013FF, "AL", "Albania"},
    {0x501C00, 0x501FFF, "HR", "Croatia"},
    {0x502C00, 0x502FFF, "LV", "Latvia"},
    {0x503C00, 0x503FFF, "LT", "Lithuania"},
    {0x504C00, 0x504FFF, "MD", "Moldova"},
    {0x505C00, 0x505FFF, "SK", "Slovakia"},
    {0x506C00, 0x506FFF, "SI", "Slovenia"},
    {0x507C00, 0x507FFF, "UZ", "Uzbekistan"},
    {0x508000, 0x50FFFF, "UA", "Ukraine"},
    {0x510000, 0x5103FF, "BY", "Belarus"},
    {0x511000, 0x5113FF, "EE", "Estonia"},
    {0x512000, 0x5123FF, "MK", "North Macedonia"},
    {0x513000, 0x5133FF, "BA", "Bosnia and Herzegovina"},
    {0x514000, 0x5143FF, "GE", "Georgia"},
    {0x515000, 0x5153FF, "TJ", "Tajikistan"},
    {0x516000, 0x5163FF, "ME", "Montenegro"},
    {0x600000, 0x6003FF, "AM", "Armenia"},
    {0x600800, 0x600BFF, "AZ", "Azerbaijan"},
    {0x601000, 0x6013FF, "KG", "Kyrgyzstan"},
    {0x601800, 0x601BFF, "TM", "Turkmenistan"},
    {0x680000, 0x6803FF, "BT", "Bhutan"},
    {0x681000, 0x6813FF, "FM", "Micronesia"},
    {0x682000, 0x6823FF, "MN", "Mongolia"},
    {0x683000, 0x6833FF, "KZ", "Kazakhstan"},
    {0x684000, 0x6843FF, "PW", "Palau"},
    {0x700000, 0x700FFF, "AF", "Afghanistan"},
    {0x702000, 0x702FFF, "BD", "Bangladesh"},
    {0x704000, 0x704FFF, "MM", "Myanmar"},
    {0x706000, 0x706FFF, "KW", "Kuwait"},
    {0x708000, 0x708FFF, "LA", "Laos"},
    {0x70A000, 0x70AFFF, "NP", "Nepal"},
    {0x70C000, 0x70C3FF, "OM", "Oman"},
    {0x70E000, 0x70EFFF, "KH", "Cambodia"},
    {0x710000, 0x717FFF, "SA", "Saudi Arabia"},
    {0x718000, 0x71FFFF, "KR", "South Korea"},
    {0x720000, 0x727FFF, "KP", "North Korea"},
    {0x728000, 0x72FFFF, "IQ", "Iraq"},
    {0x730000, 0x737FFF, "IR", "Iran"},
    {0x738000, 0x73FFFF, "IL", "Israel"},
    {0x740000, 0x747FFF, "JO", "Jordan"},
    {0x748000, 0x74FFFF, "LB", "Lebanon"},
    {0x750000, 0x757FFF, "MY", "Malaysia"},
    {0x758000, 0x75FFFF, "PH", "Philippines"},
    {0x760000, 0x767FFF, "PK", "Pakistan"},
    {0x768000, 0x76FFFF, "SG", "Singapore"},
    {0x770000, 0x777FFF, "LK", "Sri Lanka"},
    {0x778000, 0x77FFFF, "SY", "Syria"},
    {0x780000, 0x7BFFFF, "CN", "China"},
    {0x7C0000, 0x7FFFFF, "AU", "Australia"},
    {0x800000, 0x83FFFF, "IN", "India"},
    {0x840000, 0x87FFFF, "JP", "Japan"},
    {0x880000, 0x887FFF, "TH", "Thailand"},
    {0x888000, 0x88FFFF, "VN", "Vietnam"},
    {0x890000, 0x890FFF, "YE", "Yemen"},
    {0x894000, 0x894FFF, "BH", "Bahrain"},
    {0x895000, 0x8953FF, "BN", "Brunei"},
    {0x896000, 0x896FFF, "AE", "United Arab Emirates"},
    {0x897000, 0x8973FF, "TL", "Timor-Leste"},
    {0x898000, 0x898FFF, "PS", "Palestine"},
    {0x899000, 0x8993FF, "MH", "Marshall Islands"},
    {0x8A0000, 0x8A7FFF, "ID", "Indonesia"},
    {0x900000, 0x9003FF, "SB", "Solomon Islands"},
    {0x901000, 0x9013FF, "PG", "Papua New Guinea"},
    {0x902000, 0x9023FF, "VU", "Vanuatu"},
    {0x903000, 0x9033FF, "FJ", "Fiji"},
    {0x908000, 0x9083FF, "NR", "Nauru"},
    {0xA00000, 0xAFFFFF, "US", "United States"},
    {0xC00000, 0xC3FFFF, "CA", "Canada"},
    {0xC80000, 0xC87FFF, "NZ", "New Zealand"},
    {0xC88000, 0xC88FFF, "WS", "Samoa"},
    {0xC8A000, 0xC8A3FF, "TO", "Tonga"},
    {0xE00000, 0xE3FFFF, "AR", "Argentina"},
    {0xE40000, 0xE7FFFF, "BR", "Brazil"},
    {0xE80000, 0xE80FFF, "CL", "Chile"},
    {0xE84000, 0xE84FFF, "EC", "Ecuador"},
    {0xE88000, 0xE88FFF, "PY", "Paraguay"},
    {0xE8C000, 0xE8CFFF, "PE", "Peru"},
    {0xE90000, 0xE90FFF, "UY", "Uruguay"},
    {0xE94000, 0xE94FFF, "BO", "Bolivia"},
    {0xF00000, 0xF07FFF, "RU", "Russia"},
    {0xF09000, 0xF093FF, "TW", "Taiwan"},
};

static constexpr size_t k_icao_country_db_size =
    sizeof(k_icao_country_db) / sizeof(k_icao_country_db[0]);

// binary search: returns pointer to matching entry or nullptr
inline const IcaoCountryEntry* icao_lookup_country(uint32_t icao)
{
    // find the last entry whose start <= icao
    auto it = std::upper_bound(
        k_icao_country_db, k_icao_country_db + k_icao_country_db_size, icao,
        [](uint32_t addr, const IcaoCountryEntry& e) { return addr < e.start; });

    if (it == k_icao_country_db) return nullptr;
    --it;
    if (icao >= it->start && icao <= it->end)
        return it;
    return nullptr;
}

// Resolve a full country name from an ISO-3166-1 alpha-2 code (e.g. "DE"
// -> "Germany"). Used by the airport tooltip. Linear scan is fine — ~200
// entries and the call only fires on tooltip render.
inline const char* country_name_from_iso(const char* iso2)
{
    if (!iso2 || !iso2[0] || !iso2[1]) return nullptr;
    char up[2] = {
        static_cast<char>(std::toupper(static_cast<unsigned char>(iso2[0]))),
        static_cast<char>(std::toupper(static_cast<unsigned char>(iso2[1]))),
    };
    for (size_t i = 0; i < k_icao_country_db_size; ++i) {
        if (k_icao_country_db[i].code[0] == up[0] &&
            k_icao_country_db[i].code[1] == up[1]) {
            return k_icao_country_db[i].name;
        }
    }
    return nullptr;
}

// human-readable category description from packed category byte
inline const char* adsb_category_description(uint8_t cat)
{
    switch (cat) {
        case 0x00: return "No category info";
        // set A (cat >> 4 == 0xA or encoded as 0xA0..0xA7)
        case 0xA0: return "No ADS-B emitter info";
        case 0xA1: return "Light (< 15500 lbs)";
        case 0xA2: return "Small (15500-75000 lbs)";
        case 0xA3: return "Large (75000-300000 lbs)";
        case 0xA4: return "High vortex large";
        case 0xA5: return "Heavy (> 300000 lbs)";
        case 0xA6: return "High performance (> 5g, > 400 kts)";
        case 0xA7: return "Rotorcraft";
        // set B
        case 0xB0: return "No ADS-B emitter info";
        case 0xB1: return "Glider / sailplane";
        case 0xB2: return "Lighter-than-air";
        case 0xB3: return "Parachutist / skydiver";
        case 0xB4: return "Ultralight / hang-glider";
        case 0xB5: return "Reserved";
        case 0xB6: return "Unmanned aerial vehicle";
        case 0xB7: return "Space vehicle";
        // set C
        case 0xC0: return "No ADS-B emitter info";
        case 0xC1: return "Emergency vehicle";
        case 0xC2: return "Service vehicle";
        case 0xC3: return "Point obstacle";
        case 0xC4: return "Cluster obstacle";
        case 0xC5: return "Line obstacle";
        // set D
        case 0xD0: return "No ADS-B emitter info";
        case 0xD1: return "Fixed ground obstruction";
        default: return "";
    }
}

// aircraft type classification: 0=civilian, 1=military, 2=unknown
inline int adsb_classify_type(uint8_t category, uint8_t db_flags)
{
    constexpr uint8_t DB_FLAG_MILITARY = 0x01;
    if (db_flags & DB_FLAG_MILITARY) return 1;
    switch (category) {
        case 0xA6: return 1;  // high performance
        case 0xA1: case 0xA2: case 0xA3: case 0xA4:
        case 0xA5: case 0xA7:
        case 0xC1: case 0xC2:
        case 0xB1: case 0xB2: case 0xB4: case 0xB7:
            return 0;  // civilian
        case 0xB6: return 2;  // UAV — unknown
        default: return 2;    // unknown
    }
}
